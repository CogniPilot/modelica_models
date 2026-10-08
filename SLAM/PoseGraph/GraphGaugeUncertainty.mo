within SLAM.PoseGraph;
// Conditional first-order bound transport, not graph covariance certification.
// ENU additive position and body right-local attitude; selected order p,theta,p,theta.
// Anchor/relative cross-correlation is UNKNOWN, never silently set to zero.
// sourceRevision is a positive session identity token, not a verified hash here.
// Positive provenance tokens assert caller-provided bounds; they certify nothing.
package GraphGaugeUncertainty
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;

  constant Integer chartENUPositionRightLocalAttitude = 1;
  constant Integer selectedDimension = 12;
  constant Integer anchorDimension = 6;
  record Binding
    Integer generation; Integer graphRevision; Integer catalogPoseRevision;
    Integer anchorId; Integer anchorEpoch; Integer anchorCaptureSequence;
    Integer currentId; Integer currentEpoch; Integer currentCaptureSequence; Integer referenceId; Integer referenceEpoch;
    Integer referenceCaptureSequence; Integer chart;
    Real anchorTime; Real currentTime; Real referenceTime;
    Integer sourceRevision; Integer factorProvenance; Integer anchorBoundProvenance;
  end Binding;
  record Estimate
    Binding binding;
    Real positions[2,3]; Real rotations[2,3,3]; Real covariance[12,12];
  end Estimate;
  function SameBinding
    input Binding a; input Binding b; output Boolean same;
  algorithm
    same := a.generation == b.generation and a.graphRevision == b.graphRevision
      and a.catalogPoseRevision == b.catalogPoseRevision and a.anchorId == b.anchorId
      and a.anchorEpoch == b.anchorEpoch and a.anchorCaptureSequence == b.anchorCaptureSequence
      and a.currentId == b.currentId and a.currentEpoch == b.currentEpoch and a.currentCaptureSequence == b.currentCaptureSequence
      and a.referenceId == b.referenceId and a.referenceEpoch == b.referenceEpoch
      and a.referenceCaptureSequence == b.referenceCaptureSequence and a.chart == b.chart
      and a.anchorTime == b.anchorTime and a.currentTime == b.currentTime and a.referenceTime == b.referenceTime
      and a.sourceRevision == b.sourceRevision and a.factorProvenance == b.factorProvenance
      and a.anchorBoundProvenance == b.anchorBoundProvenance;
  end SameBinding;
  // A repeated selected ID is one random pose, never two observations.
  function SameCapture
    input Binding b; output Boolean same;
  algorithm
    same := b.currentId == b.referenceId and b.currentEpoch == b.referenceEpoch
      and b.currentTime == b.referenceTime and b.currentCaptureSequence == b.referenceCaptureSequence;
  end SameCapture;
  function CloneEstimate
    input Estimate estimate; output Boolean valid;
  algorithm
    valid := SameCapture(estimate.binding);
    for i in 1:3 loop
      valid := valid and estimate.positions[1,i] == estimate.positions[2,i];
      for j in 1:3 loop valid := valid and estimate.rotations[1,i,j] == estimate.rotations[2,i,j]; end for;
    end for;
    for i in 1:6 loop for j in 1:6 loop
      valid := valid and estimate.covariance[i,j] == estimate.covariance[i,j+6]
        and estimate.covariance[i,j] == estimate.covariance[i+6,j]
        and estimate.covariance[i,j] == estimate.covariance[i+6,j+6];
    end for; end for;
  end CloneEstimate;
  function ValidBinding
    input Binding b; output Boolean valid;
  algorithm
    valid := b.generation > 0 and b.graphRevision > 0 and b.catalogPoseRevision >= 0
      and b.anchorId > 0 and b.anchorEpoch >= 0 and b.anchorCaptureSequence > 0
      and b.currentId > 0 and b.currentCaptureSequence > 0 and b.referenceId > 0 and (b.currentId <> b.referenceId or SameCapture(b))
      and b.currentEpoch >= b.referenceEpoch and b.referenceEpoch >= b.anchorEpoch
      and b.referenceCaptureSequence > 0 and b.chart == chartENUPositionRightLocalAttitude
      and b.anchorTime >= 0 and b.anchorTime <= b.referenceTime and b.referenceTime <= b.currentTime
      and b.currentTime <= 1e12 and b.sourceRevision > 0 and b.factorProvenance > 0
      and b.anchorBoundProvenance > 0;
    if b.currentId == b.anchorId then
      valid := valid and b.currentEpoch == b.anchorEpoch and b.currentTime == b.anchorTime
        and b.currentCaptureSequence == b.anchorCaptureSequence;
    end if;
    if b.referenceId == b.anchorId then
      valid := valid and b.referenceEpoch == b.anchorEpoch and b.referenceTime == b.anchorTime
        and b.referenceCaptureSequence == b.anchorCaptureSequence;
    end if;
  end ValidBinding;
  function Transport
    input Estimate previous;
    input Binding binding; input Binding expectedBinding;
    input Real anchorPosition[3]; input Real anchorRotation[3,3]; input Real anchorBound[6,6];
    input Real relativePositions[2,3]; input Real relativeRotations[2,3,3];
    input Real relativeJointBound[12,12]; input Real beta; input Boolean requested;
    output Estimate result; output Boolean accepted; output Integer rejectionReason;
  protected
    Boolean valid; Boolean endpoint;
    Real B[12,6]; Real C[12,12]; Real candidate[12,12]; Real lever[3,3];
    Real candidatePositions[2,3]; Real candidateRotations[2,3,3];
    Integer offset;
  algorithm
    result := previous; accepted := false; rejectionReason := 1;
    if requested then
      rejectionReason := 2;
      valid := ValidBinding(binding) and SameBinding(binding,expectedBinding);
      if valid then
        rejectionReason := 3; valid := beta > 0 and beta < 1;
      end if;
      if valid then
        rejectionReason := 4; valid := RGBDUncertaintyProper(anchorRotation);
        for i in 1:3 loop valid := valid and abs(anchorPosition[i]) <= 1e6; end for;
        for node in 1:2 loop
          valid := valid and RGBDUncertaintyProper(relativeRotations[node,:,:]);
          for i in 1:3 loop valid := valid and abs(relativePositions[node,i]) <= 1e6; end for;
        end for;
      end if;
      if valid then
        rejectionReason := 5;
        valid := SLAMCovariancePSDCheck(anchorBound,1e-12) == 1
          and SLAMCovariancePSDCheck(relativeJointBound,1e-12) == 1;
      end if;
      if valid then
        rejectionReason := 6;
        for node in 1:2 loop
          endpoint := (if node == 1 then binding.currentId else binding.referenceId) == binding.anchorId;
          offset := 6*(node-1);
          if endpoint then
            for i in 1:3 loop
              valid := valid and relativePositions[node,i] == 0;
              for j in 1:3 loop valid := valid and relativeRotations[node,i,j] == (if i == j then 1 else 0); end for;
            end for;
            for i in 1:6 loop for j in 1:12 loop
              valid := valid and relativeJointBound[offset+i,j] == 0 and relativeJointBound[j,offset+i] == 0;
            end for; end for;
          end if;
        end for;
      end if;
      if valid and SameCapture(binding) then
        rejectionReason := 6;
        for i in 1:3 loop
          valid := valid and relativePositions[1,i] == relativePositions[2,i];
          for j in 1:3 loop valid := valid and relativeRotations[1,i,j] == relativeRotations[2,i,j]; end for;
        end for;
        for i in 1:6 loop for j in 1:6 loop
          valid := valid and relativeJointBound[i,j] == relativeJointBound[i,j+6]
            and relativeJointBound[i,j] == relativeJointBound[i+6,j]
            and relativeJointBound[i,j] == relativeJointBound[i+6,j+6];
        end for; end for;
      end if;
      if valid then
        B := zeros(12,6); C := zeros(12,12);
        for node in 1:2 loop
          offset := 6*(node-1); lever := -anchorRotation*RGBDUncertaintySkew(relativePositions[node,:]);
          for i in 1:3 loop for j in 1:3 loop
            B[offset+i,j] := if i == j then 1 else 0; B[offset+i,j+3] := lever[i,j];
            B[offset+i+3,j+3] := relativeRotations[node,j,i];
            C[offset+i,offset+j] := anchorRotation[i,j]; C[offset+i+3,offset+j+3] := if i == j then 1 else 0;
          end for; end for;
        end for;
        candidate := B*anchorBound*transpose(B)/beta+C*relativeJointBound*transpose(C)/(1-beta);
        rejectionReason := 7;
        valid := SLAMCovariancePSDCheck(candidate,1e-12) == 1;
        if valid then
          rejectionReason := 8;
          for node in 1:2 loop
            candidatePositions[node,:] := anchorPosition+anchorRotation*relativePositions[node,:];
            candidateRotations[node,:,:] := anchorRotation*relativeRotations[node,:,:];
            valid := valid and RGBDUncertaintyProper(candidateRotations[node,:,:]);
            for i in 1:3 loop valid := valid and abs(candidatePositions[node,i]) <= 1e6; end for;
          end for;
          if valid then
            result.binding := binding; result.covariance := candidate;
            result.positions := candidatePositions; result.rotations := candidateRotations;
            accepted := true; rejectionReason := 0;
          end if;
        end if;
      end if;
    end if;
  end Transport;
end GraphGaugeUncertainty;
