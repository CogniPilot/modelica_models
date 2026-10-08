within SLAM.PoseGraph;
// Source-owned graph correction and persistent session state. These functions
// compose the actual optimizer, selected bound, gauge transport and atomic
// filter/map commit; the host supplies neither a solved graph nor a proposal.
package RGBDGraphProcessing
  import GraphGaugeUncertainty = SLAM.PoseGraph.GraphGaugeUncertainty;
  import OptimizeModelicaPoseGraph = SLAM.PoseGraph.OptimizeModelicaPoseGraph;
  import RGBDGraphAnchorBound = SLAM.PoseGraph.RGBDGraphAnchorBound;
  import RGBDGraphCaptureLedger = SLAM.PoseGraph.RGBDGraphCaptureLedger;
  import RGBDGraphEstimatorCommit = SLAM.PoseGraph.RGBDGraphEstimatorCommit;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDGraphSelectedGauge = SLAM.PoseGraph.RGBDGraphSelectedGauge;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLocalizationCatalog = SLAM.Localization.RGBDLocalizationCatalog;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDVisualVocabulary = SLAM.LoopClosure.RGBDVisualVocabulary;
  import SchmidtGraphPoseCorrection = SLAM.Inertial.SchmidtGraphPoseCorrection;

  constant Integer nodeCapacity = RGBDKeyframes.keyframeCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record State
    RGBDGraphEstimatorCommit.State estimator;
    RGBDVisualVocabulary.State vocabulary "Frozen appearance owner shared by every retained histogram";
    RGBDGraphCaptureLedger.State captures;
    SchmidtGraphPoseCorrection.Attempt attempt;
    RGBDGraphAnchorBound.Estimate anchor;
    GraphGaugeUncertainty.Estimate selected;
  end State;

  record Policy
    SchmidtGraphPoseCorrection.Policy filter;
    Integer maximumIterations;
    Integer maximumPCG;
    Integer maximumBacktracks;
    Real initialDamping;
    Real maximumPositionStep;
    Real maximumAngleStep;
    Real pcgTolerance;
    Integer covariancePCG;
    Real covarianceTolerance;
    Real anchorWeight;
    Real anchorPositionLimit;
    Real anchorAngleLimit;
    Real transportWeight;
    Real consistencyTolerance;
    Real maximumConfidence;
  end Policy;

  record Result
    State next;
    Boolean accepted;
    Integer reason "0 accepted;1 idle;2 owner/chronology;3 graph problem;4 optimizer;5 anchor;6 selection;7 commit";
    Boolean attempted;
    Integer commitReason;
    Integer filterReason;
    Integer mapReason;
    Integer anchorReason;
    Integer selectionReason;
    Integer covarianceStatus;
    Real optimizerStatus;
    Real costBefore;
    Real costAfter;
    Real acceptedIterations;
    Real pcgIterations;
    Integer projectedCount;
    Integer prunedCount;
    Boolean roundoffCertified;
  end Result;

  function DefaultPolicy
    output Policy result;
  algorithm
    result.filter := SchmidtGraphPoseCorrection.DefaultPolicy();
    result.maximumIterations := 8;
    result.maximumPCG := 48;
    result.maximumBacktracks := 8;
    result.initialDamping := 1e-3;
    result.maximumPositionStep := 0.5;
    result.maximumAngleStep := 0.15;
    result.pcgTolerance := 1e-6;
    result.covariancePCG := 48;
    result.covarianceTolerance := 1e-10;
    result.anchorWeight := 0.5;
    result.anchorPositionLimit := 5.0;
    result.anchorAngleLimit := 0.35;
    result.transportWeight := 0.5;
    result.consistencyTolerance := 1e-6;
    result.maximumConfidence := 8;
  end DefaultPolicy;

  function EmptySelected
    input Integer generation;
    input Integer sourceRevision;
    output GraphGaugeUncertainty.Estimate result;
  algorithm
    result.binding.generation := generation;
    result.binding.sourceRevision := sourceRevision;
    result.binding.graphRevision := 0;
    result.binding.catalogPoseRevision := 0;
    result.binding.anchorId := 0;
    result.binding.anchorEpoch := -1;
    result.binding.anchorCaptureSequence := 0;
    result.binding.currentId := 0;
    result.binding.currentEpoch := -1;
    result.binding.currentCaptureSequence := 0;
    result.binding.referenceId := 0;
    result.binding.referenceEpoch := -1;
    result.binding.referenceCaptureSequence := 0;
    result.binding.chart := GraphGaugeUncertainty.chartENUPositionRightLocalAttitude;
    result.binding.anchorTime := 0;
    result.binding.currentTime := 0;
    result.binding.referenceTime := 0;
    result.binding.factorProvenance := 0;
    result.binding.anchorBoundProvenance := 0;
    result.positions := zeros(2,dimension);
    result.covariance := zeros(12,12);
    for node in 1:2 loop
      result.rotations[node,:,:] := identity(dimension);
    end for;
  end EmptySelected;

  function Empty
    input RGBDLocalizationCatalog.State localization;
    output State result;
  algorithm
    // A fresh constructor is not a restore operation. Reload carries the
    // complete State, including corrected means and consumed attempts.
    assert(RGBDLocalizationCatalog.ValidHeader(localization)
      and RGBDLocalizationCatalog.ValidEstimator(localization.estimator) and not localization.initialized
      and localization.steps == 0 and localization.catalog.nextId == 1,
      "GraphProcessing.Empty requires a fresh empty localization owner");
    result.estimator := RGBDGraphEstimatorCommit.FromLocalization(localization);
    result.vocabulary := RGBDVisualVocabulary.Empty(localization.generation,
      localization.sourceRevision,localization.catalog.vocabularyVersion);
    result.captures := RGBDGraphCaptureLedger.Empty(localization.generation,localization.sourceRevision);
    result.attempt.generation := localization.generation;
    result.attempt.graphRevision := 0;
    result.attempt.factorProvenance := 0;
    result.anchor := RGBDGraphAnchorBound.Empty(localization.generation,localization.sourceRevision);
    result.selected := EmptySelected(localization.generation,localization.sourceRevision);
  end Empty;

  function Valid
    input State state;
    output Boolean valid;
  algorithm
    valid := RGBDLocalizationCatalog.ValidHeader(state.estimator.localization)
      and RGBDLocalizationCatalog.ValidEstimator(state.estimator.localization.estimator)
      and RGBDVisualVocabulary.Valid(state.vocabulary)
      and state.vocabulary.generation == state.estimator.localization.generation
      and state.vocabulary.sourceRevision == state.estimator.localization.sourceRevision
      and state.vocabulary.version == state.estimator.localization.catalog.vocabularyVersion
      and (state.estimator.localization.catalog.nextId == 1 or state.vocabulary.ready)
      and RGBDGraphEstimatorCommit.ValidView(state.estimator.localization.catalog,
        state.estimator.poses,state.estimator.localization.sourceRevision)
      and RGBDGraphCaptureLedger.Valid(state.captures,state.estimator.localization.catalog,
        state.estimator.localization.sourceRevision,state.estimator.localization.steps)
      and state.estimator.correctionRevision >= 0 and state.estimator.correctionRevision < identifierLimit
      and state.estimator.graphRevisionUsed >= 0
      and state.estimator.graphRevisionUsed <= state.estimator.localization.graph.revision
      and state.estimator.correctionRevision <= state.estimator.graphRevisionUsed
      and ((state.estimator.correctionRevision == 0) == (state.estimator.graphRevisionUsed == 0))
      and state.attempt.generation == state.estimator.localization.generation
      and state.attempt.graphRevision >= state.estimator.graphRevisionUsed
      and state.attempt.graphRevision <= state.estimator.localization.graph.revision
      and state.attempt.factorProvenance == state.attempt.graphRevision
      and state.anchor.binding.generation == state.estimator.localization.generation
      and state.anchor.binding.sourceRevision == state.estimator.localization.sourceRevision
      and state.selected.binding.generation == state.estimator.localization.generation
      and state.selected.binding.sourceRevision == state.estimator.localization.sourceRevision;
    // These are receipts of the last ACCEPTED correction. Ordinary capture
    // may advance graph/pose revisions or evict their anchor. Never treat a
    // historical receipt as a bound on today's graph: Correct regenerates it.
    if state.estimator.correctionRevision == 0 then
      valid := valid and state.anchor.binding.provenance == 0
        and state.selected.binding.graphRevision == 0;
    else
      valid := valid and GraphGaugeUncertainty.ValidBinding(state.selected.binding)
        and state.selected.binding.graphRevision == state.estimator.graphRevisionUsed
        and state.selected.binding.factorProvenance == state.estimator.graphRevisionUsed
        and state.selected.binding.anchorBoundProvenance == state.anchor.binding.provenance
        and state.anchor.binding.provenance == state.estimator.graphRevisionUsed
        and state.anchor.binding.id == state.selected.binding.anchorId
        and state.anchor.binding.id <= identifierLimit
        and state.anchor.binding.slot >= 1 and state.anchor.binding.slot <= nodeCapacity
        and state.anchor.binding.slot == mod(state.anchor.binding.id-1,nodeCapacity)+1
        and state.anchor.binding.epoch == state.selected.binding.anchorEpoch
        and state.anchor.binding.sequence == state.selected.binding.anchorCaptureSequence
        and state.anchor.binding.imageTime == state.selected.binding.anchorTime
        and state.anchor.binding.catalogPoseRevision == state.selected.binding.catalogPoseRevision
        and state.anchor.binding.catalogPoseRevision <= state.estimator.poses.revision;
    end if;
  end Valid;

  function Correct
    input State previous;
    input Policy policy;
    input Boolean requested;
    output Result result;
  protected
    Boolean valid;
    Boolean anchorAccepted;
    Boolean contextAccepted;
    Boolean changed;
    Integer oldest;
    Integer currentId;
    Integer referenceId;
    Integer anchorSlot;
    Integer currentSlot;
    Integer referenceSlot;
    Integer currentNode;
    Integer referenceNode;
    Integer contextReason;
    Integer slot;
    RGBDGraphMeasurements.Problem problem;
    RGBDGraphEstimatorCommit.Proposal proposal;
    RGBDGraphEstimatorCommit.Result committed;
    RGBDGraphSelectedGauge.Context context;
    RGBDGraphSelectedGauge.Result selected;
    RGBDGraphAnchorBound.Estimate anchor;
    GraphGaugeUncertainty.Binding binding;
    Real finalPosition[nodeCapacity,dimension];
    Real finalRotation[nodeCapacity,dimension,dimension];
    Real activeNodes;
    Real activeEdges;
  algorithm
    result.next := previous;
    result.accepted := false;
    result.reason := 1;
    result.attempted := false;
    result.commitReason := 0;
    result.filterReason := 0;
    result.mapReason := 0;
    result.anchorReason := 0;
    result.selectionReason := 0;
    result.covarianceStatus := 0;
    result.optimizerStatus := 0;
    result.costBefore := 0;
    result.costAfter := 0;
    result.acceptedIterations := 0;
    result.pcgIterations := 0;
    result.projectedCount := 0;
    result.prunedCount := 0;
    result.roundoffCertified := false;
    if requested then
      result.reason := 2;
      valid := Valid(previous) and previous.estimator.localization.initialized
        and previous.estimator.localization.catalog.nextId > 2
        and previous.estimator.localization.estimator.referenceAvailable == 1
        and previous.estimator.localization.referenceBirth.catalogId > 0
        and previous.estimator.localization.graph.revision > previous.attempt.graphRevision
        and previous.estimator.localization.lastProcessedImageEpoch == previous.estimator.localization.catalog.lastEpoch
        and previous.estimator.localization.lastProcessedImageTime == previous.estimator.localization.catalog.lastTime
        and previous.estimator.localization.predictionTime == previous.estimator.localization.catalog.lastTime
        and previous.estimator.poses.revision < identifierLimit-1;
      if valid then
        oldest := max(1,previous.estimator.localization.catalog.nextId-nodeCapacity);
        currentId := previous.estimator.localization.catalog.nextId-1;
        referenceId := previous.estimator.localization.referenceBirth.catalogId;
        anchorSlot := mod(oldest-1,nodeCapacity)+1;
        currentSlot := mod(currentId-1,nodeCapacity)+1;
        referenceSlot := mod(referenceId-1,nodeCapacity)+1;
        valid := RGBDGraphMeasurements.Bound(previous.estimator.localization.catalog,referenceId,
            referenceSlot,previous.estimator.localization.referenceBirth.epoch)
          and previous.captures.sequences[currentSlot] == previous.estimator.localization.steps
          and previous.captures.sequences[referenceSlot] == previous.estimator.localization.referenceBirth.sequence;
      end if;
      if valid then
        result.reason := 3;
        problem := RGBDGraphMeasurements.PrepareProblem(previous.estimator.localization.catalog,
          previous.estimator.localization.graph,true);
        valid := problem.accepted;
      end if;
      if valid then
        // Warm-start from the durable corrected view, not immutable raw
        // captures. The chronological fixed row is mapped by catalogSlot.
        for node in 1:nodeCapacity loop
          if node <= problem.nodeCount then
            slot := problem.catalogSlot[node];
            problem.positions[node,:] := previous.estimator.poses.positions[slot,:];
            problem.rotations[node,:,:] := previous.estimator.poses.rotations[slot,:,:];
          end if;
        end for;
        result.reason := 4;
        (finalPosition,finalRotation,result.optimizerStatus,result.costBefore,result.costAfter,
          result.acceptedIterations,result.pcgIterations,activeNodes,activeEdges) :=
          OptimizeModelicaPoseGraph(problem.positions,problem.rotations,problem.nodeMask,
            problem.edgeMask,problem.fromNode,problem.toNode,problem.measuredTranslation,
            problem.measuredRotation,problem.information,policy.maximumIterations,policy.maximumPCG,
            policy.maximumBacktracks,policy.initialDamping,policy.maximumPositionStep,
            policy.maximumAngleStep,policy.pcgTolerance);
        valid := (result.optimizerStatus == 1 or result.optimizerStatus == 2)
          and result.costAfter >= 0 and result.costAfter <= result.costBefore
          and activeNodes == problem.nodeCount and activeEdges == problem.edgeCount;
        proposal.poses := previous.estimator.poses;
        changed := false;
        for node in 1:nodeCapacity loop
          if node <= problem.nodeCount then
            slot := problem.catalogSlot[node];
            valid := valid and RGBDUncertaintyProper(finalRotation[node,:,:]);
            for axis in 1:dimension loop
              valid := valid and abs(finalPosition[node,axis]) <= 1e6;
              changed := changed or finalPosition[node,axis] <> proposal.poses.positions[slot,axis];
              for column in 1:dimension loop
                changed := changed or finalRotation[node,axis,column] <> proposal.poses.rotations[slot,axis,column];
              end for;
            end for;
            proposal.poses.positions[slot,:] := finalPosition[node,:];
            proposal.poses.rotations[slot,:,:] := finalRotation[node,:,:];
          end if;
        end for;
        // The gauge is exact, not merely close after a numerical update.
        valid := valid and max(abs(proposal.poses.positions[anchorSlot,:]
          -previous.estimator.poses.positions[anchorSlot,:])) == 0
          and max(abs(proposal.poses.rotations[anchorSlot,:,:]
          -previous.estimator.poses.rotations[anchorSlot,:,:])) == 0;
        proposal.poses.revision := previous.estimator.poses.revision+(if changed then 1 else 0);
        proposal.graphRevision := previous.estimator.localization.graph.revision;
        proposal.optimizerAccepted := valid;
      end if;
      if valid then
        result.reason := 5;
        (anchor,anchorAccepted,result.anchorReason) := RGBDGraphAnchorBound.FromCapture(
          previous.anchor,previous.estimator.localization.catalog,previous.captures,
          previous.estimator.localization.sourceRevision,previous.estimator.localization.steps,
          previous.estimator.poses.revision,proposal.poses.positions[anchorSlot,:],
          proposal.poses.rotations[anchorSlot,:,:],proposal.graphRevision,
          policy.anchorWeight,policy.anchorPositionLimit,policy.anchorAngleLimit,true);
        valid := anchorAccepted;
      end if;
      if valid then
        // Context's revision binds the input pose owner. Proposal poses may
        // advance it only at atomic Commit, after all bound checks succeed.
        context.generation := previous.estimator.localization.generation;
        context.sourceRevision := previous.estimator.localization.sourceRevision;
        context.graphRevision := 0;
        context.catalogPoseRevision := 0;
        context.captureIds := fill(0,nodeCapacity);
        context.captureSequences := fill(0,nodeCapacity);
        (context,contextAccepted,contextReason) := RGBDGraphSelectedGauge.ContextFromLedger(
          context,previous.captures,previous.estimator.localization.catalog,
          previous.estimator.localization.graph,previous.estimator.localization.sourceRevision,
          previous.estimator.localization.steps,previous.estimator.poses.revision,true);
        result.reason := 6;
        valid := contextAccepted;
      end if;
      if valid then
        currentNode := currentId-oldest+1;
        referenceNode := referenceId-oldest+1;
        binding.generation := context.generation;
        binding.sourceRevision := context.sourceRevision;
        binding.graphRevision := context.graphRevision;
        binding.catalogPoseRevision := context.catalogPoseRevision;
        binding.anchorId := oldest;
        binding.anchorEpoch := previous.captures.epochs[anchorSlot];
        binding.anchorCaptureSequence := previous.captures.sequences[anchorSlot];
        binding.anchorTime := previous.captures.times[anchorSlot];
        binding.currentId := currentId;
        binding.currentEpoch := previous.captures.epochs[currentSlot];
        binding.currentCaptureSequence := previous.captures.sequences[currentSlot];
        binding.currentTime := previous.captures.times[currentSlot];
        binding.referenceId := referenceId;
        binding.referenceEpoch := previous.captures.epochs[referenceSlot];
        binding.referenceCaptureSequence := previous.captures.sequences[referenceSlot];
        binding.referenceTime := previous.captures.times[referenceSlot];
        binding.chart := GraphGaugeUncertainty.chartENUPositionRightLocalAttitude;
        // Revision-scoped provenance is generated here and cannot be a host
        // assertion of a successful optimizer or an arbitrary covariance.
        binding.factorProvenance := proposal.graphRevision;
        binding.anchorBoundProvenance := anchor.binding.provenance;
        selected := RGBDGraphSelectedGauge.SelectFromAnchor(previous.selected,
          previous.estimator.localization.catalog,previous.estimator.localization.graph,context,
          proposal.poses.positions,proposal.poses.rotations,currentNode,referenceNode,binding,
          anchor,proposal.graphRevision,policy.transportWeight,true,
          policy.covariancePCG,policy.covarianceTolerance);
        result.selectionReason := selected.rejectionReason;
        result.covarianceStatus := selected.covarianceStatus;
        result.roundoffCertified := selected.roundoffCertified;
        valid := selected.accepted;
      end if;
      if valid then
        result.reason := 7;
        proposal.selected := selected.estimate;
        committed := RGBDGraphEstimatorCommit.Commit(previous.estimator,proposal,binding,
          previous.attempt,policy.filter,true,policy.consistencyTolerance,policy.maximumConfidence);
        result.attempted := committed.attempted;
        result.commitReason := committed.reason;
        result.filterReason := committed.filterReason;
        result.mapReason := committed.mapReason;
        // An evaluated graph/filter factor is consumed even when the filter
        // or final map refuses. Preserve the whole numerical owner on refusal.
        if committed.attempted then
          result.next.attempt := committed.nextAttempt;
        end if;
        if committed.accepted then
          result.next.estimator := committed.next;
          result.next.anchor := anchor;
          result.next.selected := selected.estimate;
          result.projectedCount := committed.projectedCount;
          result.prunedCount := committed.prunedCount;
          result.accepted := true;
          result.reason := 0;
        end if;
      end if;
    end if;
  end Correct;
end RGBDGraphProcessing;
