within Vision.Registration;
// Deterministic bounded consensus is considered only after the original fit's
// strict residual refusal. Domain, invalid-pair, rank and eigensolver refusals
// are never rescued. Clean fits retain the exact original fit and operation order.
// Added configuration: hypotheses1..1024 and consensus fraction in(0,1].
// Reason7 means no admissible stable consensus within the declared work bounds.
function FitRigidPointPairsRobust
  import FitRigidPointPairs = Vision.Registration.FitRigidPointPairs;
  import RegistrationPairResidual = Vision.Registration.RegistrationPairResidual;
  import RegistrationWhitenedResidual3 = Vision.Registration.RegistrationWhitenedResidual3;

  input Real sourcePoint[:,:];
  input Real targetPoint[:,:];
  input Real pairEnabled[:];
  input Real activeCount;
  input Real coordinateLimit;
  input Real rankTolerance;
  input Real maximumRms;
  input Integer maximumHypotheses = 64;
  input Real minimumConsensusFraction = 0.5;
  output Real accepted;
  output Real rejectionReason;
  output Real rotation[3,3];
  output Real translation[3];
  output Real validCount;
  output Real invalidCount;
  output Real rank;
  output Real cost;
  output Real rms;
  output Real eigenGap;
  output Real sourceCentroid[3];
  output Real targetCentroid[3];
  output Real finalInlierMask[size(pairEnabled,1)];
  output Real rejectedCount;
  input Boolean useCovariance = false;
  input Real sourceCovariance[size(pairEnabled,1),3,3] = zeros(size(pairEnabled,1),3,3);
  input Real targetCovariance[size(pairEnabled,1),3,3] = zeros(size(pairEnabled,1),3,3);
  input Real maximumNormalizedSquared = 9.0 "Squared radius in whitened 3D residual space";
  input Real covarianceMinimumPivot = 1e-10;
protected
  constant Integer dimension = 3;
  constant Integer maximumRefinements = 4;
  constant Integer stateModulus = 2147483647;
  constant Integer stateMultiplier = 16807;
  constant Integer stateQuotient = 127773;
  constant Integer stateRemainder = 2836;
  Boolean configurationValid; Boolean searching; Boolean stable;
  Boolean residualValid; Boolean covarianceValid; Boolean allInliers;
  Integer enabledIndices[size(pairEnabled,1)]; Integer enabledCount;
  Integer sampleIndices[dimension]; Integer lowIndex; Integer highIndex;
  Integer state; Integer candidateCount; Integer bestCount; Integer retainedCount;
  Integer minimumConsensus; Integer index;
  Real originalValidCount; Real squaredLimit; Real residual; Real squaredResidual;
  Real candidateCost; Real bestCost;
  Real fitMaximumRms; Real unusedScore;
  Real candidateMask[size(pairEnabled,1)]; Real bestMask[size(pairEnabled,1)];
  Real sampleSource[dimension,dimension]; Real sampleTarget[dimension,dimension];
  Real fitAccepted; Real fitReason; Real fitRotation[dimension,dimension]; Real fitTranslation[dimension];
  Real fitValidCount; Real fitInvalidCount; Real fitRank; Real fitCost; Real fitRms; Real fitEigenGap;
  Real fitSourceCentroid[dimension]; Real fitTargetCentroid[dimension];
algorithm
  // In calibrated mode the geometric solver estimates an unweighted rigid
  // transform; per-pair covariance certifies its residuals below. A fixed
  // metric RMS must not reject a noisy hypothesis before that test can run.
  fitMaximumRms := if useCovariance then 1e6 else maximumRms;
  (accepted,rejectionReason,rotation,translation,validCount,invalidCount,rank,cost,rms,
    eigenGap,sourceCentroid,targetCentroid) := FitRigidPointPairs(
      sourcePoint,targetPoint,pairEnabled,activeCount,coordinateLimit,rankTolerance,fitMaximumRms);
  finalInlierMask := zeros(size(pairEnabled,1)); rejectedCount := 0.0;
  configurationValid := maximumHypotheses >= 1 and maximumHypotheses <= 1024
    and minimumConsensusFraction > 0.0 and minimumConsensusFraction <= 1.0
    and maximumRms >= 0.0 and maximumRms <= 1e6;
  if useCovariance then
    configurationValid := configurationValid and maximumNormalizedSquared > 0.0
      and maximumNormalizedSquared <= 1e6 and covarianceMinimumPivot > 0.0 and covarianceMinimumPivot < 1.0;
    for pair in 1:size(pairEnabled,1) loop
      if pair <= activeCount and pairEnabled[pair] == 1.0 then
        (covarianceValid,unusedScore) := RegistrationWhitenedResidual3(zeros(3),sourceCovariance[pair,:,:],covarianceMinimumPivot);
        configurationValid := configurationValid and covarianceValid;
        (covarianceValid,unusedScore) := RegistrationWhitenedResidual3(zeros(3),targetCovariance[pair,:,:],covarianceMinimumPivot);
        configurationValid := configurationValid and covarianceValid;
      end if;
    end for;
    if accepted > 0.5 then
      allInliers := true;
      for pair in 1:size(pairEnabled,1) loop
        if pair <= activeCount and pairEnabled[pair] == 1.0 then
          (residualValid,squaredResidual) := RegistrationPairResidual(sourcePoint[pair,:],targetPoint[pair,:],
            rotation,translation,true,sourceCovariance[pair,:,:],targetCovariance[pair,:,:],covarianceMinimumPivot);
          allInliers := allInliers and residualValid and squaredResidual <= maximumNormalizedSquared;
        end if;
      end for;
      if not allInliers then
        accepted := 0.0; rejectionReason := 6.0; rotation := identity(dimension); translation := zeros(dimension);
      end if;
    end if;
  end if;
  if not configurationValid then
    accepted := 0.0; rejectionReason := 1.0; rotation := identity(dimension); translation := zeros(dimension);
  elseif accepted > 0.5 then
    for pair in 1:size(pairEnabled,1) loop
      finalInlierMask[pair] := if pair <= activeCount and pairEnabled[pair] >= 1.0 and pairEnabled[pair] <= 1.0 then 1.0 else 0.0;
    end for;
  elseif rejectionReason >= 6.0 and rejectionReason <= 6.0 then
    // The original reason6 proves all active requested pairs valid and finite.
    originalValidCount := validCount; enabledCount := 0;
    enabledIndices := fill(0,size(pairEnabled,1));
    for pair in 1:size(pairEnabled,1) loop
      if pair <= activeCount and pairEnabled[pair] >= 1.0 and pairEnabled[pair] <= 1.0 then
        enabledCount := enabledCount+1; enabledIndices[enabledCount] := pair;
      end if;
    end for;
    minimumConsensus := max(dimension,integer(ceil(minimumConsensusFraction*originalValidCount)));
    squaredLimit := if useCovariance then maximumNormalizedSquared else maximumRms*maximumRms;
    candidateMask := zeros(size(pairEnabled,1)); bestMask := zeros(size(pairEnabled,1));
    bestCount := 0; bestCost := 0.0; state := 1;
    sampleSource := zeros(dimension,dimension); sampleTarget := zeros(dimension,dimension);
    for hypothesis in 1:maximumHypotheses loop
      // Schrage's form of the31-bit Park-Miller state keeps every intermediate
      // inside signed32-bit range; no host RNG or unbounded duplicate retry.
      state := stateMultiplier*mod(state,stateQuotient)-stateRemainder*div(state,stateQuotient);
      if state <= 0 then state := state+stateModulus; end if;
      sampleIndices[1] := 1+mod(state,enabledCount);
      state := stateMultiplier*mod(state,stateQuotient)-stateRemainder*div(state,stateQuotient);
      if state <= 0 then state := state+stateModulus; end if;
      sampleIndices[2] := 1+mod(state,enabledCount-1);
      if sampleIndices[2] >= sampleIndices[1] then sampleIndices[2] := sampleIndices[2]+1; end if;
      lowIndex := min(sampleIndices[1],sampleIndices[2]); highIndex := max(sampleIndices[1],sampleIndices[2]);
      state := stateMultiplier*mod(state,stateQuotient)-stateRemainder*div(state,stateQuotient);
      if state <= 0 then state := state+stateModulus; end if;
      sampleIndices[3] := 1+mod(state,enabledCount-2);
      if sampleIndices[3] >= lowIndex then sampleIndices[3] := sampleIndices[3]+1; end if;
      if sampleIndices[3] >= highIndex then sampleIndices[3] := sampleIndices[3]+1; end if;
      for sample in 1:dimension loop
        index := enabledIndices[sampleIndices[sample]];
        for axis in 1:dimension loop
          sampleSource[sample,axis] := sourcePoint[index,axis];
          sampleTarget[sample,axis] := targetPoint[index,axis];
        end for;
      end for;
      (fitAccepted,fitReason,fitRotation,fitTranslation,fitValidCount,fitInvalidCount,fitRank,
        fitCost,fitRms,fitEigenGap,fitSourceCentroid,fitTargetCentroid) := FitRigidPointPairs(
          sampleSource,sampleTarget,ones(dimension),dimension,coordinateLimit,rankTolerance,fitMaximumRms);
      if fitAccepted > 0.5 then
        candidateCount := 0; candidateCost := 0.0;
        for compact in 1:enabledCount loop
          index := enabledIndices[compact];
          if useCovariance then
            (residualValid,squaredResidual) := RegistrationPairResidual(sourcePoint[index,:],targetPoint[index,:],
              fitRotation,fitTranslation,true,sourceCovariance[index,:,:],targetCovariance[index,:,:],covarianceMinimumPivot);
          else
            // Preserve the original clean/Euclidean operation order.
            residualValid := true; squaredResidual := 0.0;
            for axis in 1:dimension loop
              residual := fitTranslation[axis]-targetPoint[index,axis];
              for column in 1:dimension loop residual := residual+fitRotation[axis,column]*sourcePoint[index,column]; end for;
              squaredResidual := squaredResidual+residual*residual;
            end for;
          end if;
          candidateMask[index] := if residualValid and squaredResidual >= 0.0 and squaredResidual <= squaredLimit then 1.0 else 0.0;
          if candidateMask[index] > 0.5 then
            candidateCount := candidateCount+1; candidateCost := candidateCost+squaredResidual;
          end if;
        end for;
        if candidateCount > bestCount or (candidateCount == bestCount and candidateCost < bestCost) then
          bestCount := candidateCount; bestCost := candidateCost; bestMask := candidateMask;
        end if;
      end if;
    end for;
    searching := bestCount >= minimumConsensus; stable := false;
    for refinement in 1:maximumRefinements loop
      if searching then
        (fitAccepted,fitReason,fitRotation,fitTranslation,fitValidCount,fitInvalidCount,fitRank,
          fitCost,fitRms,fitEigenGap,fitSourceCentroid,fitTargetCentroid) := FitRigidPointPairs(
            sourcePoint,targetPoint,bestMask,activeCount,coordinateLimit,rankTolerance,fitMaximumRms);
        if fitAccepted > 0.5 then
          retainedCount := 0;
          for compact in 1:enabledCount loop
            index := enabledIndices[compact]; candidateMask[index] := 0.0;
            if bestMask[index] > 0.5 then
              if useCovariance then
                (residualValid,squaredResidual) := RegistrationPairResidual(sourcePoint[index,:],targetPoint[index,:],
                  fitRotation,fitTranslation,true,sourceCovariance[index,:,:],targetCovariance[index,:,:],covarianceMinimumPivot);
              else
                residualValid := true; squaredResidual := 0.0;
                for axis in 1:dimension loop
                  residual := fitTranslation[axis]-targetPoint[index,axis];
                  for column in 1:dimension loop residual := residual+fitRotation[axis,column]*sourcePoint[index,column]; end for;
                  squaredResidual := squaredResidual+residual*residual;
                end for;
              end if;
              if residualValid and squaredResidual >= 0.0 and squaredResidual <= squaredLimit then
                candidateMask[index] := 1.0; retainedCount := retainedCount+1;
              end if;
            end if;
          end for;
          stable := retainedCount == bestCount and retainedCount >= minimumConsensus;
          if stable then
            accepted := fitAccepted; rejectionReason := fitReason;
            rotation := fitRotation; translation := fitTranslation;
            validCount := fitValidCount; invalidCount := fitInvalidCount; rank := fitRank;
            cost := fitCost; rms := fitRms; eigenGap := fitEigenGap;
            sourceCentroid := fitSourceCentroid; targetCentroid := fitTargetCentroid;
            finalInlierMask := bestMask; rejectedCount := originalValidCount-fitValidCount;
            searching := false;
          else
            bestMask := candidateMask; bestCount := retainedCount;
            searching := retainedCount >= minimumConsensus;
          end if;
        else
          searching := false;
        end if;
      end if;
    end for;
    if not stable then
      accepted := 0.0; rejectionReason := 7.0;
      rotation := identity(dimension); translation := zeros(dimension);
      // Retain original full-fit diagnostics, but never publish an unproved mask.
      finalInlierMask := zeros(size(pairEnabled,1)); rejectedCount := 0.0;
    end if;
  end if;
end FitRigidPointPairsRobust;
