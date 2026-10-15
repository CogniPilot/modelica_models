within SLAM.Mapping;
// One frame proposal. Expensive retrieval/registration runs only on capture.
// Both branches remain proposals for the enclosing estimator's atomic commit.
package RGBDCatalogFrame
  import RGBDCatalogGraphCapture = SLAM.PoseGraph.RGBDCatalogGraphCapture;
  import RGBDCatalogMapping = SLAM.Mapping.RGBDCatalogMapping;
  import RGBDCatalogObservation = SLAM.Mapping.RGBDCatalogObservation;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframePolicy = SLAM.LoopClosure.RGBDKeyframePolicy;
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;

  function Advance
    input RGBDKeyframes.Catalog catalog;
    input RGBDGraphMeasurements.State graph;
    input RGBDCatalogMapping.State map;
    input RGBDKeyframes.Frame measurement;
    input Real vocabulary[RGBDKeyframes.wordCapacity,RGBDKeyframes.descriptorSize];
    input Real vocabularyEnabled[RGBDKeyframes.wordCapacity];
    input Real candidatePoint[RGBDKeyframes.featureCapacity,RGBDKeyframes.dimension];
    input Real candidateEnabled[RGBDKeyframes.featureCapacity];
    input Boolean imageFresh;
    input Real poseAccepted;
    input Boolean requested;
    input Real minimumInterval = 0.5; input Real maximumInterval = 2.0;
    input Real translationThreshold = 0.6; input Real rotationThreshold = 0.25;
    input Integer minimumFeatures = 12;
    input Integer seeds[RGBDKeyframeRetrieval.proposalCapacity] = fill(7,RGBDKeyframeRetrieval.proposalCapacity);
    input Integer sequentialSeed = 7;
    input Real maximumWordDistanceSquared = 0.8; input Integer minimumAssignments = 8;
    input Real minimumSimilarity = 0.35; input Real minimumAge = 5.0;
    input Integer trials = 96; input Integer refinements = 4;
    input Integer minimumInliers = 12; input Real minimumFraction = 0.5;
    input Real inlierDistance = 0.08; input Real maximumRms = 0.03;
    input Real descriptorRatio = 0.8; input Real maximumDescriptorDistance = 0.8;
    input Real registrationCoordinateLimit = 100.0; input Real rankTolerance = 1e-8;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
    input Real coordinateLimit = 1e6; input Real voxelWidth = 0.25;
    input Real mergeRadius = 0.15; input Real maximumDistance = 80.0;
    input Real tentativeLifetime = 0.5; input Real confirmedLifetime = 5.0;
    input Real confirmationObservations = 3.0; input Real maximumConfidence = 8.0;
    input Real maximumTentative = 700.0; input Real consistencyTolerance = 1e-6;
    input Real nodePosition[RGBDKeyframes.keyframeCapacity,3] = catalog.bodyPositions;
    input Real nodeRotation[RGBDKeyframes.keyframeCapacity,3,3] = catalog.bodyRotations;
    input Integer poseRevision = map.catalogRevision;
    input Boolean poseConsistentLoops = true "Screen repeated-place aliases against retained pose uncertainty";
    output RGBDCatalogMapping.Result result;
    output RGBDKeyframePolicy.Decision decision;
    input Real loopMinimumSimilarity = minimumSimilarity;
    input Real loopMinimumAge = minimumAge;
    input Integer loopMinimumInliers = minimumInliers;
    input Real loopMinimumFraction = minimumFraction;
    input Real loopInlierDistance = inlierDistance;
    input Real loopMaximumRms = maximumRms;
    input Boolean loopCalibratedResiduals = true;
    input Real loopMaximumNormalizedSquared = 9.0;
  protected
    RGBDCatalogGraphCapture.Result visual;
    RGBDKeyframes.Catalog policyCatalog;
  algorithm
    policyCatalog := catalog;
    policyCatalog.bodyPositions := nodePosition; policyCatalog.bodyRotations := nodeRotation;
    decision := RGBDKeyframePolicy.Select(policyCatalog,measurement,imageFresh,poseAccepted,requested,
      minimumInterval,maximumInterval,translationThreshold,rotationThreshold,minimumFeatures,coordinateLimit);
    if requested and decision.valid and decision.captureRequested then
      visual := RGBDCatalogGraphCapture.Capture(catalog,measurement,vocabulary,vocabularyEnabled,graph,
        true,seeds,sequentialSeed,maximumWordDistanceSquared,minimumAssignments,minimumSimilarity,minimumAge,
        trials,refinements,minimumInliers,minimumFraction,inlierDistance,maximumRms,descriptorRatio,
        maximumDescriptorDistance,registrationCoordinateLimit,rankTolerance,localizationSigma,depthInflation,minimumPivot,
        poseConsistentLoops=poseConsistentLoops,
        loopMinimumSimilarity=loopMinimumSimilarity, loopMinimumAge=loopMinimumAge, loopMinimumInliers=loopMinimumInliers, loopMinimumFraction=loopMinimumFraction, loopInlierDistance=loopInlierDistance, loopMaximumRms=loopMaximumRms, loopCalibratedResiduals=loopCalibratedResiduals, loopMaximumNormalizedSquared=loopMaximumNormalizedSquared);
      if visual.rejectionReason == 4 and visual.graphDiagnostics.rejectionReason == 4
          and (visual.sequentialDiagnostics.rejectionReason == 7
            or visual.sequentialDiagnostics.rejectionReason == 11) then
        // Failed geometric consensus adds no keyframe or edge. The admitted
        // observation can still update landmarks against the retained anchors.
        result := RGBDCatalogObservation.Update(catalog,graph,map,measurement,candidatePoint,candidateEnabled,
          poseAccepted,true,coordinateLimit,voxelWidth,mergeRadius,maximumDistance,
          tentativeLifetime,confirmedLifetime,confirmationObservations,maximumConfidence,maximumTentative,consistencyTolerance,
          nodePosition,nodeRotation,poseRevision);
      else
        result := RGBDCatalogMapping.Capture(catalog,graph,map,visual,candidatePoint,candidateEnabled,
          poseAccepted,true,coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,
          confirmationObservations,maximumConfidence,maximumTentative,consistencyTolerance,nodePosition,nodeRotation,poseRevision);
      end if;
      result.diagnostics.keyframeRejectionReason := visual.rejectionReason;
      result.diagnostics.sequentialRejectionReason := visual.sequentialDiagnostics.rejectionReason;
      result.diagnostics.loopAttempted := true;
      result.diagnostics.loopGraphReason := visual.graphDiagnostics.rejectionReason;
      result.diagnostics.loopRetrievalReason := visual.loopDiagnostics.retrievalRejectionReason;
      result.diagnostics.loopCandidateCount := visual.loopDiagnostics.candidateCount;
      result.diagnostics.loopCandidateReferenceId := visual.loopDiagnostics.candidateId;
      result.diagnostics.loopCandidateScore := visual.loopDiagnostics.candidateScore;
      for rank in 1:RGBDKeyframeRetrieval.proposalCapacity loop
        result.diagnostics.loopCandidateCurrentId[rank] := if rank <= visual.loopDiagnostics.candidateCount then measurement.id else 0;
        result.diagnostics.loopCandidateReason[rank] := visual.loopDiagnostics.proposals[rank].rejectionReason;
        result.diagnostics.loopCandidateMatchedCount[rank] := visual.loopDiagnostics.proposals[rank].matchedCount;
        result.diagnostics.loopCandidateInliers[rank] := visual.loopDiagnostics.proposals[rank].inlierCount;
        result.diagnostics.loopCandidateRms[rank] := visual.loopDiagnostics.proposals[rank].rms;
        result.diagnostics.loopCandidateVerified[rank] := visual.loopDiagnostics.proposals[rank].verified;
      end for;
    else
      result := RGBDCatalogObservation.Update(catalog,graph,map,measurement,candidatePoint,candidateEnabled,
        poseAccepted,requested and decision.valid,coordinateLimit,voxelWidth,mergeRadius,maximumDistance,
        tentativeLifetime,confirmedLifetime,confirmationObservations,maximumConfidence,maximumTentative,consistencyTolerance,
        nodePosition,nodeRotation,poseRevision);
    end if;
  end Advance;
end RGBDCatalogFrame;
