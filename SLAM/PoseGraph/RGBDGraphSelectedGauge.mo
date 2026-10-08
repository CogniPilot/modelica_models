within SLAM.PoseGraph;
// Numerical first-order adapter. ContextFromLedger binds capture identities to
// their durable owner; anchor-bound provenance remains an explicit input.
package RGBDGraphSelectedGauge
  import GraphGaugeUncertainty = SLAM.PoseGraph.GraphGaugeUncertainty;
  import ModelicaPoseGraphCovariance = SLAM.PoseGraph.ModelicaPoseGraphCovariance;
  import RGBDGraphAnchorBound = SLAM.PoseGraph.RGBDGraphAnchorBound;
  import RGBDGraphCaptureLedger = SLAM.PoseGraph.RGBDGraphCaptureLedger;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;

  constant Integer nodeCapacity = RGBDKeyframes.keyframeCapacity;
  constant Integer edgeCapacity = RGBDGraphMeasurements.edgeCapacity;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record Context
    Integer generation; Integer sourceRevision; Integer graphRevision; Integer catalogPoseRevision;
    Integer captureIds[nodeCapacity];
    Integer captureSequences[nodeCapacity] "Accepted processing-step birth, indexed by catalog slot";
  end Context;

  record Result
    GraphGaugeUncertainty.Estimate estimate;
    Boolean accepted;
    Integer rejectionReason "0 accepted;1 idle;2 context/graph/ledger;3 selection/binding;4 final pose/anchor;5 covariance;6 transport;7 typed anchor binding/mean";
    Integer covarianceStatus; Integer transportReason;
    Boolean roundoffCertified;
  end Result;

  function ContextFromLedger
    input Context previous;
    input RGBDGraphCaptureLedger.State ledger;
    input RGBDKeyframes.Catalog catalog;
    input RGBDGraphMeasurements.State graph;
    input Integer sourceRevision; input Integer step; input Integer catalogPoseRevision;
    input Boolean requested = true;
    output Context context; output Boolean accepted; output Integer reason;
  protected Boolean valid;
  algorithm
    context := previous; accepted := false; reason := 1;
    if requested then
      reason := 2;
      valid := RGBDGraphCaptureLedger.Valid(ledger,catalog,sourceRevision,step);
      if valid then
        reason := 3;
        valid := graph.generation == catalog.generation
          and graph.revision > 0 and graph.revision <= identifierLimit
          and catalogPoseRevision >= 0 and catalogPoseRevision < identifierLimit;
      end if;
      if valid then
        context.generation := catalog.generation; context.sourceRevision := sourceRevision;
        context.graphRevision := graph.revision; context.catalogPoseRevision := catalogPoseRevision;
        context.captureIds := ledger.ids; context.captureSequences := ledger.sequences;
        accepted := true; reason := 0;
      end if;
    end if;
  end ContextFromLedger;

  function SelectFromAnchor
    input GraphGaugeUncertainty.Estimate previous;
    input RGBDKeyframes.Catalog catalog;
    input RGBDGraphMeasurements.State graph;
    input Context context;
    input Real finalPositionsBySlot[nodeCapacity,3]; input Real finalRotationsBySlot[nodeCapacity,3,3];
    input Integer currentNode; input Integer referenceNode;
    input GraphGaugeUncertainty.Binding binding;
    input RGBDGraphAnchorBound.Estimate anchor;
    input Integer factorProvenance; input Real beta; input Boolean requested = true;
    input Integer maximumPCG = 48; input Real tolerance = 1e-10;
    output Result result;
  protected Boolean valid; Integer id; Integer slot;
  algorithm
    result.estimate := previous; result.accepted := false; result.rejectionReason := 1;
    result.covarianceStatus := 0; result.transportReason := 0; result.roundoffCertified := false;
    if requested then
      result.rejectionReason := 7;
      valid := RGBDKeyframes.ValidHeader(catalog) and catalog.nextId > 1
        and GraphGaugeUncertainty.ValidBinding(binding);
      if valid then
        id := max(1,catalog.nextId-nodeCapacity); slot := mod(id-1,nodeCapacity)+1;
        valid := anchor.binding.generation == catalog.generation
          and anchor.binding.generation == context.generation and anchor.binding.generation == binding.generation
          and anchor.binding.sourceRevision == context.sourceRevision and anchor.binding.sourceRevision == binding.sourceRevision
          and anchor.binding.id == id and anchor.binding.id == binding.anchorId
          and anchor.binding.slot == slot and catalog.occupied[slot] and catalog.ids[slot] == id
          and context.captureIds[slot] == id
          and anchor.binding.epoch == catalog.epochs[slot] and anchor.binding.epoch == binding.anchorEpoch
          and anchor.binding.sequence == context.captureSequences[slot]
          and anchor.binding.sequence == binding.anchorCaptureSequence
          and anchor.binding.catalogPoseRevision == context.catalogPoseRevision
          and anchor.binding.catalogPoseRevision == binding.catalogPoseRevision
          and anchor.binding.imageTime == catalog.imageTimes[slot] and anchor.binding.imageTime == binding.anchorTime
          and anchor.binding.provenance >= 1 and anchor.binding.provenance <= identifierLimit
          and anchor.binding.provenance == binding.anchorBoundProvenance
          and max(abs(anchor.position-finalPositionsBySlot[slot,:])) == 0
          and max(abs(anchor.rotation-finalRotationsBySlot[slot,:,:])) == 0;
      end if;
      if valid then
        // The typed owner supplies a local ERROR SECOND-MOMENT bound about
        // this exact mean; it is not a centered/global covariance certificate.
        result := SelectAndTransport(previous,catalog,graph,context,finalPositionsBySlot,finalRotationsBySlot,
          currentNode,referenceNode,binding,anchor.position,anchor.rotation,anchor.bound,
          anchor.binding.provenance,factorProvenance,beta,true,maximumPCG,tolerance);
      end if;
    end if;
  end SelectFromAnchor;

  function SelectAndTransport
    input GraphGaugeUncertainty.Estimate previous;
    input RGBDKeyframes.Catalog catalog;
    input RGBDGraphMeasurements.State graph;
    input Context context;
    input Real finalPositionsBySlot[nodeCapacity,3];
    input Real finalRotationsBySlot[nodeCapacity,3,3];
    input Integer currentNode; input Integer referenceNode;
    input GraphGaugeUncertainty.Binding binding;
    input Real anchorPosition[3]; input Real anchorRotation[3,3]; input Real anchorBound[6,6];
    input Integer anchorBoundProvenance; input Integer factorProvenance;
    input Real beta; input Boolean requested = true;
    input Integer maximumPCG = 48; input Real tolerance = 1e-10;
    output Result result;
  protected
    RGBDGraphMeasurements.Problem problem;
    ModelicaPoseGraphCovariance.Result selected;
    GraphGaugeUncertainty.Binding expected;
    GraphGaugeUncertainty.Estimate transported;
    Boolean valid; Boolean transportAccepted;
    Integer slot; Integer previousSequence; Integer anchorSlot; Integer currentSlot; Integer referenceSlot;
    Integer nodes[2]; Integer offset;
    Real relativePositions[2,3]; Real relativeRotations[2,3,3];
    Real D[12,12]; Real relativeBound[12,12];
  algorithm
    result.estimate := previous; result.accepted := false; result.rejectionReason := 1;
    result.covarianceStatus := 0; result.transportReason := 0; result.roundoffCertified := false;
    if requested then
      result.rejectionReason := 2;
      valid := RGBDKeyframes.ValidHeader(catalog)
        and context.generation == catalog.generation and graph.generation == catalog.generation
        and context.sourceRevision >= 1 and context.sourceRevision <= identifierLimit
        and context.graphRevision == graph.revision and graph.revision > 0
        and context.catalogPoseRevision >= 0 and context.catalogPoseRevision < identifierLimit
        and anchorBoundProvenance > 0 and anchorBoundProvenance <= identifierLimit
        and factorProvenance > 0 and factorProvenance <= identifierLimit;
      if valid then
        // Prepare the actual graph once; no caller-constructed Problem can
        // substitute endpoints, information scale or a different factor set.
        problem := RGBDGraphMeasurements.PrepareProblem(catalog,graph,true);
        valid := problem.accepted;
      end if;
      if valid then
        previousSequence := 0;
        for node in 1:nodeCapacity loop
          if node <= problem.nodeCount then
            slot := problem.catalogSlot[node];
            valid := valid and context.captureIds[slot] == problem.nodeId[node]
              and context.captureSequences[slot] > previousSequence
              and context.captureSequences[slot] <= identifierLimit;
            previousSequence := context.captureSequences[slot];
          end if;
        end for;
      end if;
      if valid then
        result.rejectionReason := 3;
        valid := currentNode >= 1 and currentNode <= problem.nodeCount
          and referenceNode >= 1 and referenceNode <= problem.nodeCount
          and (currentNode <> referenceNode or GraphGaugeUncertainty.SameCapture(binding));
        if valid then
          anchorSlot := problem.catalogSlot[1]; currentSlot := problem.catalogSlot[currentNode];
          referenceSlot := problem.catalogSlot[referenceNode];
          expected.generation := catalog.generation; expected.sourceRevision := context.sourceRevision;
          expected.graphRevision := graph.revision; expected.catalogPoseRevision := context.catalogPoseRevision;
          expected.anchorId := problem.nodeId[1]; expected.anchorEpoch := catalog.epochs[anchorSlot];
          expected.anchorTime := catalog.imageTimes[anchorSlot];
          expected.anchorCaptureSequence := context.captureSequences[anchorSlot];
          expected.currentId := problem.nodeId[currentNode]; expected.currentEpoch := catalog.epochs[currentSlot];
          expected.currentTime := catalog.imageTimes[currentSlot];
          expected.currentCaptureSequence := context.captureSequences[currentSlot];
          expected.referenceId := problem.nodeId[referenceNode]; expected.referenceEpoch := catalog.epochs[referenceSlot];
          expected.referenceTime := catalog.imageTimes[referenceSlot];
          expected.referenceCaptureSequence := context.captureSequences[referenceSlot];
          expected.chart := GraphGaugeUncertainty.chartENUPositionRightLocalAttitude;
          expected.factorProvenance := factorProvenance; expected.anchorBoundProvenance := anchorBoundProvenance;
          valid := GraphGaugeUncertainty.ValidBinding(expected) and GraphGaugeUncertainty.SameBinding(binding,expected);
        end if;
      end if;
      if valid then
        result.rejectionReason := 4;
        for node in 1:nodeCapacity loop
          if node <= problem.nodeCount then
            slot := problem.catalogSlot[node];
            valid := valid and RGBDUncertaintyProper(finalRotationsBySlot[slot,:,:]);
            for axis in 1:3 loop valid := valid and abs(finalPositionsBySlot[slot,axis]) <= 1e6; end for;
            problem.positions[node,:] := finalPositionsBySlot[slot,:];
            problem.rotations[node,:,:] := finalRotationsBySlot[slot,:,:];
          end if;
        end for;
        valid := valid and max(abs(anchorPosition-problem.positions[1,:])) == 0
          and max(abs(anchorRotation-problem.rotations[1,:,:])) == 0;
      end if;
      if valid then
        result.rejectionReason := 5;
        selected := ModelicaPoseGraphCovariance.Select(problem.positions,problem.rotations,problem.nodeMask,
          problem.edgeMask,problem.fromNode,problem.toNode,problem.measuredTranslation,problem.measuredRotation,
          problem.information,currentNode,referenceNode,maximumPCG,tolerance,true);
        result.covarianceStatus := selected.status;
        valid := selected.accepted;
      end if;
      if valid then
        nodes := {currentNode,referenceNode}; D := zeros(12,12);
        for node in 1:2 loop
          offset := 6*(node-1);
          // The gauge endpoint is an exact identity/zero, including the
          // selected solver's exact-zero covariance rows and columns.
          if nodes[node] == 1 then
            relativePositions[node,:] := zeros(3); relativeRotations[node,:,:] := identity(3);
          else
            relativePositions[node,:] := transpose(anchorRotation)*(problem.positions[nodes[node],:]-anchorPosition);
            relativeRotations[node,:,:] := transpose(anchorRotation)*problem.rotations[nodes[node],:,:];
          end if;
          for row in 1:3 loop for column in 1:3 loop
            D[offset+row,offset+column] := anchorRotation[column,row];
            D[offset+row+3,offset+column+3] := if row == column then 1 else 0;
          end for; end for;
        end for;
        // Rotate every position-position, position-angle and cross-pose cell.
        relativeBound := D*selected.upper*transpose(D);
        result.rejectionReason := 6;
        (transported,transportAccepted,result.transportReason) := GraphGaugeUncertainty.Transport(
          previous,binding,expected,anchorPosition,anchorRotation,anchorBound,
          relativePositions,relativeRotations,relativeBound,beta,true);
        if transportAccepted then
          result.estimate := transported; result.accepted := true; result.rejectionReason := 0;
          result.roundoffCertified := selected.roundoffCertified;
        end if;
      end if;
    end if;
  end SelectAndTransport;
end RGBDGraphSelectedGauge;
