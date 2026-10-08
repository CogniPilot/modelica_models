within SLAM.Mapping;
// Per-landmark conditional first-order bound. No landmark/filter independence
// or joint-landmark guarantee. Local geometry remains an immutable input.
package RGBDMapAnchorUncertainty
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;

  constant Integer anchorCapacity = 128;
  constant Integer landmarkCapacity = 14400;
  constant Integer positionDimension = 3;
  constant Integer anchorDimension = 6;
  constant Integer identifierLimit = 1000000000;
  record Binding
    Integer generation; Integer sourceRevision; Integer landmarkSlot;
    Integer anchorId; Integer anchorSlot; Integer anchorEpoch; Integer anchorPoseRevision;
    Integer rawObservationEpoch; Integer rawObservationProvenance;
    Integer localBoundProvenance; Integer anchorBoundProvenance;
    Real anchorTime; Real observationTime;
  end Binding;
  record Estimate
    Binding binding; Real worldPoint[3]; Real covariance[3,3];
  end Estimate;
  function SameBinding
    input Binding a; input Binding b; output Boolean same;
  algorithm
    same := a.generation == b.generation and a.sourceRevision == b.sourceRevision
      and a.landmarkSlot == b.landmarkSlot and a.anchorId == b.anchorId and a.anchorSlot == b.anchorSlot
      and a.anchorEpoch == b.anchorEpoch and a.anchorPoseRevision == b.anchorPoseRevision
      and a.rawObservationEpoch == b.rawObservationEpoch and a.rawObservationProvenance == b.rawObservationProvenance
      and a.localBoundProvenance == b.localBoundProvenance and a.anchorBoundProvenance == b.anchorBoundProvenance
      and a.anchorTime == b.anchorTime and a.observationTime == b.observationTime;
  end SameBinding;
  function ValidBinding
    input Binding b; output Boolean valid;
  algorithm
    valid := b.generation > 0 and b.sourceRevision > 0 and b.landmarkSlot >= 1 and b.landmarkSlot <= landmarkCapacity
      and b.anchorId > 0 and b.anchorId <= identifierLimit and b.anchorSlot >= 1 and b.anchorSlot <= anchorCapacity
      and b.anchorEpoch >= 0 and b.anchorPoseRevision >= 0 and b.rawObservationEpoch >= b.anchorEpoch
      and b.rawObservationProvenance > 0 and b.localBoundProvenance > 0 and b.anchorBoundProvenance > 0
      and b.anchorTime >= 0 and b.observationTime <= 1e12
      and (if b.rawObservationEpoch == b.anchorEpoch then b.observationTime == b.anchorTime
        else b.observationTime > b.anchorTime);
    if valid then
      // Certify domains before subtraction/modulo. Slot reuse never preserves
      // a stable anchor identity, even when the caller repeats bad metadata.
      valid := b.anchorSlot == mod(b.anchorId-1,anchorCapacity)+1;
    end if;
  end ValidBinding;
  function Transport
    input Estimate previous; input Binding binding; input Binding expectedBinding;
    input Real anchorPosition[3]; input Real anchorRotation[3,3]; input Real anchorBound[6,6];
    input Real localPoint[3]; input Real localBound[3,3]; input Real beta; input Boolean requested;
    output Estimate result; output Boolean accepted; output Integer rejectionReason;
  protected
    Boolean valid; Real J[3,6]; Real lever[3,3]; Real point[3]; Real covariance[3,3];
  algorithm
    result := previous; accepted := false; rejectionReason := 1;
    if requested then
      rejectionReason := 2; valid := ValidBinding(binding) and SameBinding(binding,expectedBinding);
      if valid then rejectionReason := 3; valid := beta > 0 and beta < 1; end if;
      if valid then
        rejectionReason := 4; valid := RGBDUncertaintyProper(anchorRotation);
        for i in 1:3 loop valid := valid and abs(anchorPosition[i]) <= 1e6 and abs(localPoint[i]) <= 1e6; end for;
      end if;
      if valid then
        rejectionReason := 5;
        valid := SLAMCovariancePSDCheck(anchorBound,1e-12) == 1 and SLAMCovariancePSDCheck(localBound,1e-12) == 1;
      end if;
      if valid then
        J := zeros(3,6); J[:,1:3] := identity(3); lever := -anchorRotation*RGBDUncertaintySkew(localPoint); J[:,4:6] := lever;
        point := anchorPosition+anchorRotation*localPoint;
        covariance := J*anchorBound*transpose(J)/beta+anchorRotation*localBound*transpose(anchorRotation)/(1-beta);
        rejectionReason := 6;
        valid := SLAMCovariancePSDCheck(covariance,1e-12) == 1;
        for i in 1:3 loop valid := valid and abs(point[i]) <= 1e6; end for;
        if valid then
          result.binding := binding; result.worldPoint := point; result.covariance := covariance;
          accepted := true; rejectionReason := 0;
        end if;
      end if;
    end if;
  end Transport;
end RGBDMapAnchorUncertainty;
