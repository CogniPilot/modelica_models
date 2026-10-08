within SLAM.PoseGraph;
// Local first-order ERROR SECOND-MOMENT bound about a graph anchor mean.
// Raw capture means/bounds stay immutable; changing gauge never creates a zero
// absolute bound. Input capture covariance is assumed a valid local error bound.
package RGBDGraphAnchorBound
  import RGBDGraphCaptureLedger = SLAM.PoseGraph.RGBDGraphCaptureLedger;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import SLAMCovariancePSDCheck = SLAM.Inertial.SLAMCovariancePSDCheck;
  import SchmidtGraphPoseCorrection = SLAM.Inertial.SchmidtGraphPoseCorrection;

  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer poseDimension = RGBDKeyframes.poseDimension;
  constant Integer capacity = RGBDKeyframes.keyframeCapacity;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record Binding
    Integer generation; Integer sourceRevision; Integer id; Integer slot;
    Integer epoch; Integer sequence; Integer catalogPoseRevision; Integer provenance;
    Real imageTime;
  end Binding;
  record Estimate
    Binding binding;
    Real position[dimension]; Real rotation[dimension,dimension];
    Real bound[poseDimension,poseDimension];
  end Estimate;

  function Empty
    input Integer generation; input Integer sourceRevision;
    output Estimate result;
  algorithm
    result.binding.generation := generation; result.binding.sourceRevision := sourceRevision;
    result.binding.id := 0; result.binding.slot := 0; result.binding.epoch := -1;
    result.binding.sequence := 0; result.binding.catalogPoseRevision := 0;
    result.binding.provenance := 0; result.binding.imageTime := 0;
    result.position := zeros(dimension); result.rotation := identity(dimension);
    result.bound := zeros(poseDimension,poseDimension);
  end Empty;

  function FromCapture
    input Estimate previous;
    input RGBDKeyframes.Catalog catalog;
    input RGBDGraphCaptureLedger.State ledger;
    input Integer sourceRevision; input Integer step; input Integer catalogPoseRevision;
    input Real anchorPosition[dimension]; input Real anchorRotation[dimension,dimension];
    input Integer provenance;
    input Real weight = 0.5;
    input Real maximumPositionShift = 5.0; input Real maximumAngleShift = 0.35;
    input Boolean requested = true;
    output Estimate result; output Boolean accepted; output Integer reason;
  protected
    Boolean valid; Boolean chartValid; Boolean unchanged;
    Integer id; Integer slot;
    Real offset[poseDimension]; Real J[poseDimension,poseDimension];
    Real attitudeOffset[dimension]; Real angleJacobian[dimension,dimension];
    Real captureBound[poseDimension,poseDimension]; Real candidate[poseDimension,poseDimension];
  algorithm
    result := previous; accepted := false; reason := 1;
    if requested then
      reason := 2;
      valid := RGBDGraphCaptureLedger.Valid(ledger,catalog,sourceRevision,step)
        and catalog.nextId > 1
        and catalogPoseRevision >= 0 and catalogPoseRevision < identifierLimit
        and provenance >= 1 and provenance <= identifierLimit;
      id := 1; slot := 1; unchanged := false;
      if valid then
        id := max(1,catalog.nextId-capacity); slot := mod(id-1,capacity)+1;
        valid := catalog.occupied[slot] and catalog.ids[slot] == id;
      end if;
      if valid then
        reason := 3;
        valid := weight > 0 and weight < 1
          and maximumPositionShift > 0 and maximumPositionShift <= 100
          and maximumAngleShift > 0 and maximumAngleShift <= 0.35;
      end if;
      if valid then
        reason := 4;
        valid := RGBDUncertaintyProper(anchorRotation)
          and RGBDUncertaintyProper(catalog.bodyRotations[slot,:,:]);
        unchanged := true;
        for axis in 1:dimension loop
          valid := valid and abs(anchorPosition[axis]) <= 1e6
            and abs(catalog.bodyPositions[slot,axis]) <= 1e6;
          unchanged := unchanged and anchorPosition[axis] == catalog.bodyPositions[slot,axis];
          for column in 1:dimension loop
            unchanged := unchanged and anchorRotation[axis,column] == catalog.bodyRotations[slot,axis,column];
          end for;
        end for;
        captureBound := catalog.poseCovariances[slot,:,:];
        valid := valid and SLAMCovariancePSDCheck(captureBound,1e-12) == 1;
      end if;
      if valid then
        candidate := captureBound;
        if not unchanged then
          reason := 5;
          offset := zeros(poseDimension);
          offset[1:dimension] := catalog.bodyPositions[slot,:]-anchorPosition;
          (attitudeOffset,chartValid) := SchmidtGraphPoseCorrection.Log(
            transpose(anchorRotation)*catalog.bodyRotations[slot,:,:]);
          valid := chartValid and sqrt(offset[1:dimension]*offset[1:dimension]) <= maximumPositionShift
            and sqrt(attitudeOffset*attitudeOffset) <= maximumAngleShift;
          if valid then
            offset[dimension+1:poseDimension] := attitudeOffset;
            J := identity(poseDimension);
            angleJacobian := SchmidtGraphPoseCorrection.InverseJacobian(attitudeOffset,true);
            J[dimension+1:poseDimension,dimension+1:poseDimension] := angleJacobian;
            // If e_new = offset+J*e_capture to first order and E[e e']<=P,
            // weighted Young bounds its SECOND MOMENT even for unknown mean.
            candidate := J*captureBound*transpose(J)/weight
              +outerProduct(offset,offset)/(1-weight);
          end if;
        end if;
        if valid then
          reason := 6; valid := SLAMCovariancePSDCheck(candidate,1e-12) == 1;
          if valid then
            result.binding.generation := catalog.generation;
            result.binding.sourceRevision := sourceRevision; result.binding.id := id; result.binding.slot := slot;
            result.binding.epoch := catalog.epochs[slot]; result.binding.sequence := ledger.sequences[slot];
            result.binding.catalogPoseRevision := catalogPoseRevision;
            result.binding.provenance := provenance; result.binding.imageTime := catalog.imageTimes[slot];
            result.position := anchorPosition; result.rotation := anchorRotation; result.bound := candidate;
            accepted := true; reason := 0;
          end if;
        end if;
      end if;
    end if;
  end FromCapture;
end RGBDGraphAnchorBound;
