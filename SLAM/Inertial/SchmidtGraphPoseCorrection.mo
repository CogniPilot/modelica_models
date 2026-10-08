within SLAM.Inertial;
// Unknown graph/filter cross-correlation fusion; not a fresh independent image.
// Source-owned first-order math. Not selected by the production browser runtime.
// Dependencies: GraphGaugeUncertainty, RGBDUncertaintyProper/Skew,
// SLAMCovariancePSDCheck. No optimizer damping is interpreted as covariance.
package SchmidtGraphPoseCorrection
  import GraphGaugeUncertainty = SLAM.PoseGraph.GraphGaugeUncertainty;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;

  constant Integer currentDimension = 15;
  constant Integer referenceDimension = 6;
  constant Integer jointDimension = currentDimension+referenceDimension;
  constant Integer observationDimension = 2*referenceDimension;

  constant Integer poseIndices[6] = {1,2,3,7,8,9};

  record State
    Real position[3]; Real velocity[3]; Real rotation[3,3];
    Real accelBias[3]; Real gyroBias[3];
    Real covariance[15,15]; Real crossCovariance[15,6]; Real referenceCovariance[6,6];
    Real referencePosition[3]; Real referenceRotation[3,3];
    Boolean referenceAvailable; Boolean referenceUsed;
    Integer generation; Integer sourceRevision;
    Integer currentId; Integer currentEpoch; Integer currentCaptureSequence;
    Integer referenceId; Integer referenceEpoch; Integer referenceCaptureSequence;
    Integer lastUsedEpoch;
    Real predictionTime; Real referenceTime;
  end State;

  record Policy
    Real weight; Boolean constrainVelocityAndBias;
    Real maximumPositionInnovation; Real maximumAngularInnovation;
    Real maximumPositionCorrection; Real maximumAngularCorrection; Real maximumNis;
  end Policy;

  record Attempt
    Integer generation; Integer graphRevision; Integer factorProvenance;
  end Attempt;

  record Result
    State next;
    Boolean accepted; Integer reason;
    Boolean attempted; Attempt nextAttempt;
    Real innovation[12]; Real measurementJacobian[12,21]; Real noise[12,12];
    Real gain[21,12]; Real correction[21]; Real nis;
    Real covarianceBeforeReset[21,21]; Real resetJacobian[21,21];
  end Result;

  function ExactClone
    input State state; output Boolean valid;
  algorithm
    valid := state.referenceAvailable;
    for i in 1:3 loop
      valid := valid and state.position[i] == state.referencePosition[i];
      for j in 1:3 loop valid := valid and state.rotation[i,j] == state.referenceRotation[i,j]; end for;
    end for;
    for i in 1:15 loop for j in 1:6 loop
      valid := valid and state.crossCovariance[i,j] == state.covariance[i,poseIndices[j]];
    end for; end for;
    for i in 1:6 loop for j in 1:6 loop
      valid := valid and state.referenceCovariance[i,j] == state.covariance[poseIndices[i],poseIndices[j]];
    end for; end for;
  end ExactClone;

  function DefaultPolicy
    output Policy result;
  algorithm
    result.weight := 0.5; result.constrainVelocityAndBias := false;
    result.maximumPositionInnovation := 5; result.maximumAngularInnovation := 0.35;
    result.maximumPositionCorrection := 5; result.maximumAngularCorrection := 0.35;
    result.maximumNis := 36;
  end DefaultPolicy;

  function Exp
    input Real vector[3]; output Real rotation[3,3];
  protected
    Real square; Real angle; Real a; Real b; Real skew[3,3];
  algorithm
    square := sum(vector.^2); angle := sqrt(square);
    a := if square < 1e-8 then 1-square/6+square^2/120 else sin(angle)/angle;
    b := if square < 1e-8 then 0.5-square/24+square^2/720 else (1-cos(angle))/square;
    skew := RGBDUncertaintySkew(vector);
    rotation := identity(3)+a*skew+b*(skew*skew);
  end Exp;

  function Log
    input Real rotation[3,3]; output Real vector[3]; output Boolean valid;
  protected
    Real cosine; Real angle; Real square; Real factor;
  algorithm
    vector := zeros(3); valid := RGBDUncertaintyProper(rotation);
    if valid then
      cosine := min(1.0,max(-1.0,(sum(rotation[i,i] for i in 1:3)-1)/2));
      angle := acos(cosine); square := angle^2;
      valid := angle < 3.140592653589793;
      if valid then
        factor := if square < 1e-8 then 0.5+square/12+7*square^2/720 else angle/(2*sin(angle));
        vector := factor*{rotation[3,2]-rotation[2,3],rotation[1,3]-rotation[3,1],rotation[2,1]-rotation[1,2]};
      end if;
    end if;
  end Log;

  function InverseJacobian
    input Real vector[3]; input Boolean right; output Real jacobian[3,3];
  protected
    Real square; Real angle; Real coefficient; Real skew[3,3];
  algorithm
    square := sum(vector.^2); angle := sqrt(square);
    coefficient := if square < 1e-8 then 1.0/12+square/720+square^2/30240
      else (1-angle*cos(angle/2)/(2*sin(angle/2)))/square;
    skew := RGBDUncertaintySkew(vector);
    jacobian := identity(3)+(if right then 0.5 else -0.5)*skew+coefficient*(skew*skew);
  end InverseJacobian;

  function RightJacobian
    input Real vector[3]; output Real jacobian[3,3];
  protected
    Real square; Real angle; Real a; Real b; Real skew[3,3];
  algorithm
    square := sum(vector.^2); angle := sqrt(square);
    a := if square < 1e-8 then 0.5-square/24+square^2/720 else (1-cos(angle))/square;
    b := if square < 1e-8 then 1.0/6-square/120+square^2/5040 else (angle-sin(angle))/(square*angle);
    skew := RGBDUncertaintySkew(vector);
    jacobian := identity(3)-a*skew+b*(skew*skew);
  end RightJacobian;

  // SPD12,22 RHS, dimensionless equilibration. A strictly positive pivot is
  // required; no retained jitter, diagonal floor or inverse approximation.
  function Solve
    input Real A[12,12]; input Real B[12,22];
    output Real X[12,22]; output Boolean valid;
  protected
    Real scale[12]; Real lower[12,12]; Real forward[12]; Real backward[12];
    Real value; Real residual; Real magnitude;
  algorithm
    X := zeros(12,22); lower := zeros(12,12); scale := ones(12);
    forward := zeros(12); backward := zeros(12); valid := true;
    for i in 1:12 loop
      valid := valid and A[i,i] > 0 and A[i,i] <= 1e12;
      if A[i,i] > 0 and A[i,i] <= 1e12 then scale[i] := sqrt(A[i,i]); end if;
      for j in 1:12 loop
        valid := valid and abs(A[i,j]) <= 1e12
          and abs(A[i,j]-A[j,i]) <= 1e-12*max(1.0,abs(A[i,j])+abs(A[j,i]));
      end for;
      for rhs in 1:22 loop valid := valid and abs(B[i,rhs]) <= 1e12; end for;
    end for;
    if valid then
      for i in 1:12 loop
        for j in 1:i loop
          value := A[i,j]/scale[i]/scale[j];
          for k in 1:j-1 loop value := value-lower[i,k]*lower[j,k]; end for;
          if j == i then
            valid := valid and value > 0 and value <= 1e12;
            lower[i,i] := sqrt(if value > 0 and value <= 1e12 then value else 1.0);
          else
            lower[i,j] := value/lower[j,j];
          end if;
        end for;
      end for;
    end if;
    if valid then
      for rhs in 1:22 loop
        for i in 1:12 loop
          value := B[i,rhs]/scale[i];
          for j in 1:i-1 loop value := value-lower[i,j]*forward[j]; end for;
          forward[i] := value/lower[i,i];
        end for;
        for reverse in 1:12 loop
          value := forward[13-reverse];
          for j in 14-reverse:12 loop value := value-lower[j,13-reverse]*backward[j]; end for;
          backward[13-reverse] := value/lower[13-reverse,13-reverse];
        end for;
        for i in 1:12 loop X[i,rhs] := backward[i]/scale[i]; end for;
      end for;
      for i in 1:12 loop for rhs in 1:22 loop
        residual := -B[i,rhs]; magnitude := abs(B[i,rhs]);
        for j in 1:12 loop
          residual := residual+A[i,j]*X[j,rhs]; magnitude := magnitude+abs(A[i,j]*X[j,rhs]);
        end for;
        valid := valid and abs(X[i,rhs]) <= 1e12 and abs(residual) <= 1e-9*max(1.0,magnitude);
      end for; end for;
    end if;
    if not valid then X := zeros(12,22); end if;
  end Solve;

  function Correct
    input State previous;
    input GraphGaugeUncertainty.Estimate graph;
    input GraphGaugeUncertainty.Binding expectedBinding;
    input Attempt previousAttempt;
    input Policy policy;
    input Boolean requested;
    output Result result;
  protected
    Boolean valid; Boolean logValid; Boolean solveValid; Boolean sameCapture;
    Integer count; Integer offset; Integer stateOffset;
    Real prior[21,21]; Real weightedPrior[21,21]; Real Q[12,12]; Real noiseMap[12,12];
    Real H[12,21]; Real innovation[12]; Real graphRotation[3,3]; Real priorRotation[3,3];
    Real logVector[3]; Real leftInverse[3,3]; Real rightInverse[3,3];
    Real cross[21,12]; Real S[12,12]; Real rhs[12,22]; Real solved[12,22];
    Real K[21,12]; Real delta[21]; Real A[21,21]; Real beforeReset[21,21];
    Real reset[21,21]; Real proposedCovariance[21,21]; State candidate;
  algorithm
    result.next := previous; result.accepted := false; result.reason := 1;
    result.attempted := false; result.nextAttempt := previousAttempt;
    result.innovation := zeros(12); result.measurementJacobian := zeros(12,21);
    result.noise := zeros(12,12); result.gain := zeros(21,12); result.correction := zeros(21);
    result.nis := 0; result.covarianceBeforeReset := zeros(21,21); result.resetJacobian := identity(21);
    if requested then
      sameCapture := GraphGaugeUncertainty.SameCapture(graph.binding);
      result.reason := 2;
      valid := GraphGaugeUncertainty.ValidBinding(graph.binding)
        and GraphGaugeUncertainty.SameBinding(graph.binding,expectedBinding)
        and previous.generation == graph.binding.generation
        and previous.sourceRevision == graph.binding.sourceRevision
        and previous.currentId == graph.binding.currentId and previous.currentEpoch == graph.binding.currentEpoch
        and previous.currentCaptureSequence == graph.binding.currentCaptureSequence
        and previous.predictionTime == graph.binding.currentTime
        and (previous.referenceAvailable or not previous.referenceUsed)
        and previousAttempt.generation == previous.generation and previousAttempt.graphRevision >= 0
        and (if previousAttempt.graphRevision == 0 then previousAttempt.factorProvenance == 0
          else previousAttempt.factorProvenance > 0)
        and graph.binding.graphRevision > previousAttempt.graphRevision
        and graph.binding.factorProvenance <> previousAttempt.factorProvenance;
      if previous.referenceAvailable then
        valid := valid and previous.referenceId == graph.binding.referenceId
          and previous.referenceEpoch == graph.binding.referenceEpoch
          and previous.referenceCaptureSequence == graph.binding.referenceCaptureSequence
          and previous.referenceTime == graph.binding.referenceTime
          and previous.lastUsedEpoch >= -1 and previous.lastUsedEpoch <= previous.currentEpoch
          and (if previous.referenceUsed then previous.referenceEpoch <= previous.lastUsedEpoch
            else previous.referenceEpoch > previous.lastUsedEpoch);
      end if;
      if sameCapture then valid := valid and previous.referenceAvailable; end if;
      if valid then
        // Attempt metadata survives numerical refusal. The enclosing owner must
        // persist it separately from the rolled-back numerical tuple.
        result.attempted := true;
        result.nextAttempt.generation := previous.generation;
        result.nextAttempt.graphRevision := graph.binding.graphRevision;
        result.nextAttempt.factorProvenance := graph.binding.factorProvenance;
        result.reason := 3;
        valid := policy.weight > 0 and policy.weight < 1
          and policy.maximumPositionInnovation > 0 and policy.maximumPositionInnovation <= 1e6
          and policy.maximumAngularInnovation > 0 and policy.maximumAngularInnovation < 3.140592653589793
          and policy.maximumPositionCorrection > 0 and policy.maximumPositionCorrection <= 1e6
          and policy.maximumAngularCorrection > 0 and policy.maximumAngularCorrection < 3.140592653589793
          and policy.maximumNis > 0 and policy.maximumNis <= 1e12;
      end if;
      if valid then
        result.reason := 4;
        valid := RGBDUncertaintyProper(previous.rotation) and RGBDUncertaintyProper(graph.rotations[1,:,:]);
        for i in 1:3 loop
          valid := valid and abs(previous.position[i]) <= 1e6 and abs(previous.velocity[i]) <= 1e6
            and abs(previous.accelBias[i]) <= 2 and abs(previous.gyroBias[i]) <= 0.3
            and abs(graph.positions[1,i]) <= 1e6;
        end for;
        if previous.referenceAvailable then
          valid := valid and RGBDUncertaintyProper(previous.referenceRotation)
            and RGBDUncertaintyProper(graph.rotations[2,:,:]);
          for i in 1:3 loop
            valid := valid and abs(previous.referencePosition[i]) <= 1e6 and abs(graph.positions[2,i]) <= 1e6;
          end for;
        end if;
      end if;
      if valid then
        result.reason := 5; prior := zeros(21,21);
        prior[1:15,1:15] := previous.covariance;
        if previous.referenceAvailable then
          prior[1:15,16:21] := previous.crossCovariance;
          prior[16:21,1:15] := transpose(previous.crossCovariance);
          prior[16:21,16:21] := previous.referenceCovariance;
          valid := SLAMCovariancePSDCheck(prior,1e-12) == 1
            and SLAMCovariancePSDCheck(graph.covariance,1e-12) == 1;
        else
          valid := SLAMCovariancePSDCheck(previous.covariance,1e-12) == 1
            and SLAMCovariancePSDCheck(graph.covariance[1:6,1:6],1e-12) == 1;
        end if;
      end if;
      if valid and sameCapture then
        result.reason := 5;
        valid := ExactClone(previous) and GraphGaugeUncertainty.CloneEstimate(graph);
      end if;
      if valid then
        result.reason := 6; H := zeros(12,21); noiseMap := zeros(12,12); innovation := zeros(12);
        count := if previous.referenceAvailable and not sameCapture then 2 else 1;
        for node in 1:count loop
          offset := 6*(node-1); stateOffset := if node == 1 then 0 else 15;
          priorRotation := if node == 1 then previous.rotation else previous.referenceRotation;
          graphRotation := graph.rotations[node,:,:];
          (logVector,logValid) := Log(transpose(priorRotation)*graphRotation);
          valid := valid and logValid and sum(logVector.^2) <= policy.maximumAngularInnovation^2;
          innovation[offset+1:offset+3] := graph.positions[node,:]
            -(if node == 1 then previous.position else previous.referencePosition);
          innovation[offset+4:offset+6] := logVector;
          valid := valid and sum(innovation[offset+1:offset+3].^2) <= policy.maximumPositionInnovation^2;
          leftInverse := InverseJacobian(logVector,false); rightInverse := InverseJacobian(logVector,true);
          H[offset+1:offset+3,stateOffset+1:stateOffset+3] := identity(3);
          H[offset+4:offset+6,stateOffset+(if node == 1 then 7 else 4):stateOffset+(if node == 1 then 9 else 6)] := leftInverse;
          noiseMap[offset+1:offset+3,offset+1:offset+3] := identity(3);
          noiseMap[offset+4:offset+6,offset+4:offset+6] := rightInverse;
        end for;
      end if;
      if valid then
        Q := zeros(12,12);
        if previous.referenceAvailable and not sameCapture then
          Q := noiseMap*graph.covariance*transpose(noiseMap);
        else
          Q[1:6,1:6] := noiseMap[1:6,1:6]*graph.covariance[1:6,1:6]*transpose(noiseMap[1:6,1:6]);
          // Neutral solver padding for current-only or one same-capture pose,
          // neither an extra reference observation nor retained
          // covariance. Zero H/K rows make this identical to the six-row solve.
          Q[7:12,7:12] := identity(6);
        end if;
        weightedPrior := prior/policy.weight; cross := weightedPrior*transpose(H);
        S := H*cross+Q/(1-policy.weight);
        rhs[:,1:21] := transpose(cross); rhs[:,22] := innovation;
        (solved,solveValid) := Solve(S,rhs);
        result.reason := 7; valid := solveValid;
      end if;
      if valid then
        K := transpose(solved[:,1:21]);
        if policy.constrainVelocityAndBias then
          K[4:6,:] := zeros(3,12); K[10:15,:] := zeros(6,12);
        end if;
        delta := K*innovation;
        result.innovation := innovation; result.measurementJacobian := H; result.noise := Q;
        result.gain := K; result.correction := delta; result.nis := sum(innovation.*solved[:,22]);
        result.reason := 8;
        valid := result.nis >= 0 and result.nis <= policy.maximumNis;
        for i in 1:21 loop valid := valid and abs(delta[i]) <= 1e6; end for;
        valid := valid and sum(delta[1:3].^2) <= policy.maximumPositionCorrection^2
          and sum(delta[7:9].^2) <= policy.maximumAngularCorrection^2;
        if previous.referenceAvailable then
          valid := valid and sum(delta[16:18].^2) <= policy.maximumPositionCorrection^2
            and sum(delta[19:21].^2) <= policy.maximumAngularCorrection^2;
        end if;
      end if;
      if valid then
        candidate := previous;
        candidate.position := previous.position+delta[1:3]; candidate.velocity := previous.velocity+delta[4:6];
        candidate.rotation := previous.rotation*Exp(delta[7:9]);
        candidate.accelBias := previous.accelBias+delta[10:12]; candidate.gyroBias := previous.gyroBias+delta[13:15];
        if previous.referenceAvailable then
          candidate.referencePosition := previous.referencePosition+delta[16:18];
          candidate.referenceRotation := previous.referenceRotation*Exp(delta[19:21]);
        end if;
        result.reason := 9; valid := RGBDUncertaintyProper(candidate.rotation);
        for i in 1:3 loop
          valid := valid and abs(candidate.position[i]) <= 1e6 and abs(candidate.velocity[i]) <= 1e6
            and abs(candidate.accelBias[i]) <= 2 and abs(candidate.gyroBias[i]) <= 0.3;
        end for;
        if previous.referenceAvailable then
          valid := valid and RGBDUncertaintyProper(candidate.referenceRotation);
          for i in 1:3 loop valid := valid and abs(candidate.referencePosition[i]) <= 1e6; end for;
        end if;
      end if;
      if valid then
        A := identity(21)-K*H;
        beforeReset := A*weightedPrior*transpose(A)+K*(Q/(1-policy.weight))*transpose(K);
        reset := identity(21); reset[7:9,7:9] := RightJacobian(delta[7:9]);
        if previous.referenceAvailable then reset[19:21,19:21] := RightJacobian(delta[19:21]); end if;
        proposedCovariance := reset*beforeReset*transpose(reset);
        proposedCovariance := (proposedCovariance+transpose(proposedCovariance))/2;
        result.covarianceBeforeReset := beforeReset; result.resetJacobian := reset;
        result.reason := 10;
        valid := SLAMCovariancePSDCheck(proposedCovariance,1e-12) == 1;
        if valid and sameCapture then
          // Validate full21 result closure before the structural clone map
          // C=[I15; selected6]. Only NEW corrected Pcc/means are retained.
          for i in 1:3 loop
            valid := valid and abs(candidate.referencePosition[i]-candidate.position[i])
              <= 1e-11*(1+max(abs(candidate.referencePosition[i]),abs(candidate.position[i])));
            for j in 1:3 loop valid := valid and abs(candidate.referenceRotation[i,j]-candidate.rotation[i,j]) <= 1e-11; end for;
          end for;
          for i in 1:15 loop for j in 1:6 loop
            valid := valid and abs(proposedCovariance[i,j+15]-proposedCovariance[i,poseIndices[j]])
              <= 1e-11*(1+max(abs(proposedCovariance[i,j+15]),abs(proposedCovariance[i,poseIndices[j]])));
          end for; end for;
          for i in 1:6 loop for j in 1:6 loop
            valid := valid and abs(proposedCovariance[i+15,j+15]-proposedCovariance[poseIndices[i],poseIndices[j]])
              <= 1e-11*(1+max(abs(proposedCovariance[i+15,j+15]),abs(proposedCovariance[poseIndices[i],poseIndices[j]])));
          end for; end for;
          if valid then
            candidate.referencePosition := candidate.position; candidate.referenceRotation := candidate.rotation;
            for i in 1:15 loop for j in 1:6 loop
              proposedCovariance[i,j+15] := proposedCovariance[i,poseIndices[j]];
              proposedCovariance[j+15,i] := proposedCovariance[poseIndices[j],i];
            end for; end for;
            for i in 1:6 loop for j in 1:6 loop
              proposedCovariance[i+15,j+15] := proposedCovariance[poseIndices[i],poseIndices[j]];
            end for; end for;
            valid := SLAMCovariancePSDCheck(proposedCovariance,1e-12) == 1;
          end if;
        end if;
        if valid then
          candidate.covariance := proposedCovariance[1:15,1:15];
          if previous.referenceAvailable then
            candidate.crossCovariance := proposedCovariance[1:15,16:21];
            candidate.referenceCovariance := proposedCovariance[16:21,16:21];
          end if;
          result.next := candidate; result.accepted := true; result.reason := 0;
        end if;
      end if;
    end if;
  end Correct;
end SchmidtGraphPoseCorrection;
