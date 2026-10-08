within SLAM.PoseGraph;
// Staged graph/map/filter publication. Raw captures and measured edges never
// change during correction. Graph pose means have a separate versioned owner.
// Geometry-only map outputs are not independent filter measurements.
package RGBDGraphEstimatorCommit
  import GraphGaugeUncertainty = SLAM.PoseGraph.GraphGaugeUncertainty;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLandmarkCatalog = SLAM.Mapping.RGBDLandmarkCatalog;
  import RGBDLocalizationCatalog = SLAM.Localization.RGBDLocalizationCatalog;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import SchmidtGraphPoseCorrection = SLAM.Inertial.SchmidtGraphPoseCorrection;

  constant Integer nodeCapacity = RGBDKeyframes.keyframeCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record PoseView
    Integer generation; Integer sourceRevision; Integer revision; Integer catalogNextId;
    Boolean enabled[nodeCapacity]; Integer ids[nodeCapacity];
    Real positions[nodeCapacity,dimension]; Real rotations[nodeCapacity,dimension,dimension];
  end PoseView;

  record State
    RGBDLocalizationCatalog.State localization;
    PoseView poses;
    Integer correctionRevision; Integer graphRevisionUsed;
  end State;

  record Proposal
    PoseView poses;
    GraphGaugeUncertainty.Estimate selected;
    Integer graphRevision;
    Boolean optimizerAccepted;
  end Proposal;

  record Result
    State next;
    Boolean accepted; Integer reason;
    Boolean attempted; SchmidtGraphPoseCorrection.Attempt nextAttempt;
    Boolean filterAccepted; Integer filterReason;
    Integer mapReason; Integer reprojectionReason;
    Integer projectedCount; Integer prunedCount;
  end Result;

  function FromLocalization
    input RGBDLocalizationCatalog.State localization;
    output State result;
  algorithm
    // Bootstrap only. Reconstructing this view from raw captures after a graph
    // correction would erase accepted pose means and is not a restore operation.
    result.localization := localization;
    result.poses.generation := localization.generation;
    result.poses.sourceRevision := localization.sourceRevision;
    result.poses.revision := 0; result.poses.catalogNextId := localization.catalog.nextId;
    result.poses.enabled := localization.catalog.occupied; result.poses.ids := localization.catalog.ids;
    result.poses.positions := localization.catalog.bodyPositions;
    result.poses.rotations := localization.catalog.bodyRotations;
    result.correctionRevision := 0; result.graphRevisionUsed := 0;
  end FromLocalization;

  function ValidView
    input RGBDKeyframes.Catalog catalog;
    input PoseView poses;
    input Integer sourceRevision;
    output Boolean valid;
  algorithm
    valid := RGBDKeyframes.ValidHeader(catalog)
      and poses.generation == catalog.generation and poses.sourceRevision == sourceRevision
      and sourceRevision >= 1 and sourceRevision <= identifierLimit
      and poses.revision >= 0 and poses.revision < identifierLimit
      and poses.catalogNextId == catalog.nextId;
    for slot in 1:nodeCapacity loop
      valid := valid and poses.enabled[slot] == catalog.occupied[slot];
      if poses.enabled[slot] then
        valid := valid and poses.ids[slot] == catalog.ids[slot]
          and RGBDUncertaintyProper(poses.rotations[slot,:,:]);
        for axis in 1:dimension loop valid := valid and abs(poses.positions[slot,axis]) <= 1e6; end for;
      end if;
    end for;
  end ValidView;

  function EstimatorState
    input RGBDLocalizationCatalog.State localization;
    output SchmidtGraphPoseCorrection.State result;
  protected
    Integer slot;
  algorithm
    // Caller validates headers, exact Real integer domains and reference birth
    // before this conversion; no projection, registration or host algebra.
    result.position := localization.estimator.position; result.velocity := localization.estimator.velocity;
    result.rotation := localization.estimator.rotation;
    result.accelBias := localization.estimator.accelBias; result.gyroBias := localization.estimator.gyroBias;
    result.covariance := localization.estimator.covariance;
    result.crossCovariance := localization.estimator.crossCovariance;
    result.referenceCovariance := localization.estimator.referenceCovariance;
    result.referencePosition := localization.estimator.referencePosition;
    result.referenceRotation := localization.estimator.referenceRotation;
    result.referenceAvailable := localization.estimator.referenceAvailable == 1;
    result.referenceUsed := localization.estimator.referenceUsed == 1;
    result.generation := localization.generation; result.sourceRevision := localization.sourceRevision;
    result.currentId := localization.catalog.nextId-1;
    result.currentEpoch := localization.lastProcessedImageEpoch;
    result.currentCaptureSequence := localization.steps;
    result.referenceId := localization.referenceBirth.catalogId;
    result.referenceEpoch := localization.referenceBirth.epoch;
    result.referenceCaptureSequence := localization.referenceBirth.sequence;
    result.lastUsedEpoch := integer(localization.estimator.lastUsedEpoch);
    result.predictionTime := localization.predictionTime; result.referenceTime := 0;
    if result.referenceAvailable then
      slot := mod(result.referenceId-1,nodeCapacity)+1;
      result.referenceTime := localization.catalog.imageTimes[slot];
    end if;
  end EstimatorState;

  function Commit
    input State previous;
    input Proposal proposal;
    input GraphGaugeUncertainty.Binding expectedBinding;
    input SchmidtGraphPoseCorrection.Attempt previousAttempt;
    input SchmidtGraphPoseCorrection.Policy policy;
    input Boolean requested;
    input Real consistencyTolerance = 1e-6;
    input Real maximumConfidence = 8;
    output Result result;
  protected
    Boolean valid; Boolean changed; Boolean mapAccepted;
    Integer currentSlot; Integer referenceSlot; Integer anchorSlot;
    SchmidtGraphPoseCorrection.State filterState;
    SchmidtGraphPoseCorrection.Result filterResult;
    State candidate;
  algorithm
    result.next := previous; result.accepted := false; result.reason := 1;
    result.attempted := false; result.nextAttempt := previousAttempt;
    result.filterAccepted := false; result.filterReason := 0;
    result.mapReason := 0; result.reprojectionReason := 0;
    result.projectedCount := 0; result.prunedCount := 0;
    if requested then
      result.reason := 2;
      valid := RGBDLocalizationCatalog.ValidHeader(previous.localization)
        and previous.localization.initialized
        and RGBDLocalizationCatalog.ValidEstimator(previous.localization.estimator)
        and ValidView(previous.localization.catalog,previous.poses,previous.localization.sourceRevision)
        and previous.correctionRevision >= 0 and previous.correctionRevision < identifierLimit
        and previous.graphRevisionUsed >= 0 and previous.graphRevisionUsed <= previous.localization.graph.revision
        and previous.correctionRevision <= previous.graphRevisionUsed
        and ((previous.correctionRevision == 0) == (previous.graphRevisionUsed == 0))
        and previous.localization.catalog.nextId > 2
        and previous.localization.lastProcessedImageEpoch == previous.localization.catalog.lastEpoch
        and previous.localization.lastProcessedImageTime == previous.localization.catalog.lastTime
        and previous.localization.predictionTime == previous.localization.catalog.lastTime
        and GraphGaugeUncertainty.ValidBinding(proposal.selected.binding)
        and GraphGaugeUncertainty.SameBinding(proposal.selected.binding,expectedBinding)
        and proposal.graphRevision == previous.localization.graph.revision
        and expectedBinding.graphRevision == proposal.graphRevision
        and expectedBinding.catalogPoseRevision == previous.poses.revision
        and expectedBinding.generation == previous.localization.generation
        and expectedBinding.sourceRevision == previous.localization.sourceRevision
        and expectedBinding.currentId == previous.localization.catalog.nextId-1
        and expectedBinding.currentEpoch == previous.localization.lastProcessedImageEpoch
        and expectedBinding.currentTime == previous.localization.predictionTime
        and expectedBinding.currentCaptureSequence == previous.localization.steps
        and expectedBinding.anchorId == max(1,previous.localization.catalog.nextId-nodeCapacity)
        and previousAttempt.generation == previous.localization.generation
        and previousAttempt.graphRevision >= previous.graphRevisionUsed
        and (if previousAttempt.graphRevision == 0 then previousAttempt.factorProvenance == 0
          else previousAttempt.factorProvenance > 0)
        and proposal.graphRevision > previousAttempt.graphRevision
        and proposal.selected.binding.factorProvenance <> previousAttempt.factorProvenance;
      if valid then
        currentSlot := mod(expectedBinding.currentId-1,nodeCapacity)+1;
        referenceSlot := mod(expectedBinding.referenceId-1,nodeCapacity)+1;
        anchorSlot := mod(expectedBinding.anchorId-1,nodeCapacity)+1;
        valid := RGBDGraphMeasurements.Bound(previous.localization.catalog,expectedBinding.referenceId,
            referenceSlot,expectedBinding.referenceEpoch)
          and RGBDGraphMeasurements.Bound(previous.localization.catalog,expectedBinding.anchorId,
            anchorSlot,expectedBinding.anchorEpoch)
          and expectedBinding.referenceTime == previous.localization.catalog.imageTimes[referenceSlot]
          and expectedBinding.anchorTime == previous.localization.catalog.imageTimes[anchorSlot];
        if previous.localization.estimator.referenceAvailable == 1 then
          valid := valid and previous.localization.referenceBirth.catalogId == expectedBinding.referenceId
            and previous.localization.referenceBirth.epoch == expectedBinding.referenceEpoch
            and previous.localization.referenceBirth.sequence == expectedBinding.referenceCaptureSequence;
        end if;
      end if;
      if valid then
        result.attempted := true;
        result.nextAttempt.generation := previous.localization.generation;
        result.nextAttempt.graphRevision := proposal.graphRevision;
        result.nextAttempt.factorProvenance := proposal.selected.binding.factorProvenance;
        result.reason := 3;
        valid := proposal.optimizerAccepted
          and RGBDGraphMeasurements.ValidState(previous.localization.catalog,previous.localization.graph)
          and ValidView(previous.localization.catalog,proposal.poses,previous.localization.sourceRevision)
          and consistencyTolerance > 0 and consistencyTolerance <= 1e-6
          and maximumConfidence >= 2 and maximumConfidence <= 100 and floor(maximumConfidence) == maximumConfidence;
      end if;
      if valid then
        result.reason := 4; changed := false;
        for slot in 1:nodeCapacity loop
          if previous.poses.enabled[slot] then
            for axis in 1:dimension loop
              changed := changed or proposal.poses.positions[slot,axis] <> previous.poses.positions[slot,axis];
              for column in 1:dimension loop
                changed := changed or proposal.poses.rotations[slot,axis,column] <> previous.poses.rotations[slot,axis,column];
              end for;
            end for;
          end if;
        end for;
        valid := proposal.poses.revision == previous.poses.revision+(if changed then 1 else 0);
        // The optimizer fixes the chronological first row in the same ENU gauge.
        for axis in 1:dimension loop
          valid := valid and proposal.poses.positions[anchorSlot,axis] == previous.poses.positions[anchorSlot,axis];
          for column in 1:dimension loop
            valid := valid and proposal.poses.rotations[anchorSlot,axis,column] == previous.poses.rotations[anchorSlot,axis,column];
          end for;
        end for;
        for node in 1:2 loop
          for axis in 1:dimension loop
            valid := valid and abs(proposal.selected.positions[node,axis]
              -proposal.poses.positions[if node == 1 then currentSlot else referenceSlot,axis]) <= consistencyTolerance;
            for column in 1:dimension loop
              valid := valid and abs(proposal.selected.rotations[node,axis,column]
                -proposal.poses.rotations[if node == 1 then currentSlot else referenceSlot,axis,column]) <= consistencyTolerance;
            end for;
          end for;
        end for;
      end if;
      if valid then
        filterState := EstimatorState(previous.localization);
        filterResult := SchmidtGraphPoseCorrection.Correct(filterState,proposal.selected,expectedBinding,
          previousAttempt,policy,true);
        result.filterAccepted := filterResult.accepted; result.filterReason := filterResult.reason;
        result.reason := 5; valid := filterResult.accepted;
      end if;
      if valid then
        candidate := previous; candidate.poses.revision := proposal.poses.revision;
        for slot in 1:nodeCapacity loop
          if previous.poses.enabled[slot] then
            candidate.poses.positions[slot,:] := proposal.poses.positions[slot,:];
            candidate.poses.rotations[slot,:,:] := proposal.poses.rotations[slot,:,:];
          end if;
        end for;
        candidate.localization.estimator.position := filterResult.next.position;
        candidate.localization.estimator.velocity := filterResult.next.velocity;
        candidate.localization.estimator.rotation := filterResult.next.rotation;
        candidate.localization.estimator.accelBias := filterResult.next.accelBias;
        candidate.localization.estimator.gyroBias := filterResult.next.gyroBias;
        candidate.localization.estimator.covariance := filterResult.next.covariance;
        candidate.localization.estimator.crossCovariance := filterResult.next.crossCovariance;
        candidate.localization.estimator.referenceCovariance := filterResult.next.referenceCovariance;
        candidate.localization.estimator.referencePosition := filterResult.next.referencePosition;
        candidate.localization.estimator.referenceRotation := filterResult.next.referenceRotation;
        // Preserve acquisition clocks, raw capture poses/noise/calibration,
        // every descriptor, all measured edges and the consumed-image ledger.
        (candidate.localization.map.point,candidate.localization.map.occupied,candidate.localization.map.confidence,
          candidate.localization.map.lastSeen,candidate.localization.map.lastFrame,candidate.localization.map.localPoint,
          candidate.localization.map.anchorId,candidate.localization.map.anchorSlot,
          mapAccepted,result.mapReason,result.reprojectionReason,result.projectedCount,result.prunedCount) :=
          RGBDLandmarkCatalog.Synchronize(
            previous.localization.map.point,previous.localization.map.occupied,previous.localization.map.confidence,
            previous.localization.map.lastSeen,previous.localization.map.lastFrame,previous.localization.map.localPoint,
            previous.localization.map.anchorId,previous.localization.map.anchorSlot,
            previous.localization.map.generation,previous.poses.revision,
            previous.poses.enabled,previous.poses.ids,previous.poses.positions,previous.poses.rotations,
            proposal.poses.enabled,proposal.poses.ids,proposal.poses.positions,proposal.poses.rotations,
            previous.localization.generation,proposal.poses.revision,true,true,false,
            previous.localization.map.imageTime,previous.localization.map.frame,maximumConfidence,1e6,consistencyTolerance);
        result.reason := 6; valid := mapAccepted;
        if valid then
          candidate.correctionRevision := previous.correctionRevision+1;
          candidate.graphRevisionUsed := proposal.graphRevision;
          result.reason := 7;
          valid := RGBDLocalizationCatalog.ValidHeader(candidate.localization)
            and RGBDLocalizationCatalog.ValidEstimator(candidate.localization.estimator);
          if valid then result.next := candidate; result.accepted := true; result.reason := 0; end if;
        end if;
      end if;
    end if;
    if not result.accepted then result.projectedCount := 0; result.prunedCount := 0; end if;
  end Commit;
end RGBDGraphEstimatorCommit;
