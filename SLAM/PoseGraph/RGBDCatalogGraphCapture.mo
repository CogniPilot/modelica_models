within SLAM.PoseGraph;
// Atomic proposal for the catalog and measured graph owners. Publication,
// estimator correlation, optimization and map updates remain outer owners.
package RGBDCatalogGraphCapture
  import RGBDCatalogLoopVerification = SLAM.LoopClosure.RGBDCatalogLoopVerification;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLoopVerification = SLAM.LoopClosure.RGBDLoopVerification;

  record Result
    RGBDKeyframes.Catalog catalog;
    RGBDGraphMeasurements.State graph;
    Boolean accepted;
    Integer rejectionReason "1 idle; 2 retrieval; 3 store; 4 graph; 5 problem";
    Integer storedSlot; Integer evictedId;
    RGBDCatalogLoopVerification.Batch loopDiagnostics;
    RGBDLoopVerification.Proposal sequentialDiagnostics;
    RGBDGraphMeasurements.Update graphDiagnostics;
    RGBDGraphMeasurements.Problem problem;
  end Result;

  function Capture
    input RGBDKeyframes.Catalog previousCatalog;
    input RGBDKeyframes.Frame measurement;
    input Real vocabulary[RGBDKeyframes.wordCapacity,RGBDKeyframes.descriptorSize];
    input Real vocabularyEnabled[RGBDKeyframes.wordCapacity];
    input RGBDGraphMeasurements.State previousGraph;
    input Boolean requested;
    input Integer seeds[RGBDKeyframeRetrieval.proposalCapacity] = fill(7,RGBDKeyframeRetrieval.proposalCapacity);
    input Integer sequentialSeed = 7;
    input Real maximumWordDistanceSquared = 0.8; input Integer minimumAssignments = 8;
    input Real minimumSimilarity = 0.35; input Real minimumAge = 5.0;
    input Integer trials = 96; input Integer refinements = 4;
    input Integer minimumInliers = 12; input Real minimumFraction = 0.5;
    input Real inlierDistance = 0.08; input Real maximumRms = 0.03;
    input Real descriptorRatio = 0.8; input Real maximumDescriptorDistance = 0.8;
    input Real coordinateLimit = 100.0; input Real rankTolerance = 1e-8;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
    output Result result;
    input Boolean calibratedResiduals = true;
    input Real maximumNormalizedSquared = 9.0;
    input Boolean poseConsistentLoops = false;
    input Real loopMinimumSimilarity = minimumSimilarity;
    input Real loopMinimumAge = minimumAge;
    input Integer loopMinimumInliers = minimumInliers;
    input Real loopMinimumFraction = minimumFraction;
    input Real loopInlierDistance = inlierDistance;
    input Real loopMaximumRms = maximumRms;
    input Boolean loopCalibratedResiduals = calibratedResiduals;
    input Real loopMaximumNormalizedSquared = maximumNormalizedSquared;
  protected
    RGBDKeyframes.Catalog captured;
    RGBDKeyframes.Frame latest;
    Boolean stored; Integer slot; Integer evicted; Boolean poseConsistent; Real poseSquared;
  algorithm
    result.catalog := previousCatalog; result.graph := previousGraph;
    result.accepted := false; result.rejectionReason := 1;
    result.storedSlot := 0; result.evictedId := 0;
    result.loopDiagnostics := RGBDCatalogLoopVerification.ProposeCapture(previousCatalog,measurement,vocabulary,vocabularyEnabled,
      requested,seeds,maximumWordDistanceSquared,minimumAssignments,loopMinimumSimilarity,loopMinimumAge,trials,refinements,
      loopMinimumInliers,loopMinimumFraction,loopInlierDistance,loopMaximumRms,descriptorRatio,maximumDescriptorDistance,
      coordinateLimit,rankTolerance,localizationSigma,depthInflation,minimumPivot,
      calibratedResiduals=loopCalibratedResiduals,maximumNormalizedSquared=loopMaximumNormalizedSquared,
      poseConsistentLoops=poseConsistentLoops);
    result.sequentialDiagnostics := RGBDLoopVerification.EmptyProposal(sequentialSeed);
    if result.loopDiagnostics.retrievalAccepted and previousCatalog.nextId > 1 then
      latest := RGBDKeyframes.ReadSlot(previousCatalog,mod(previousCatalog.nextId-2,RGBDKeyframes.keyframeCapacity)+1);
      result.sequentialDiagnostics := RGBDLoopVerification.Verify(latest,result.loopDiagnostics.prepared,true,
        sequentialSeed,trials,refinements,minimumInliers,minimumFraction,inlierDistance,maximumRms,0.0,
        descriptorRatio,maximumDescriptorDistance,coordinateLimit,rankTolerance,localizationSigma,depthInflation,minimumPivot,
        calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared);
      // The previous retained keyframe can also be a repeated warehouse bay
      // after a long capture gap. A false sequential edge is as damaging as
      // a false retrieved loop, so both use the same production motion screen.
      if result.sequentialDiagnostics.verified and poseConsistentLoops then
        (poseConsistent,poseSquared) := RGBDCatalogLoopVerification.PoseConsistent(
          latest.bodyPosition,measurement.bodyPosition,latest.bodyRotation,
          latest.poseCovariance,measurement.poseCovariance,
          result.sequentialDiagnostics.bodyTranslation,result.sequentialDiagnostics.covariance[1:3,1:3],minimumPivot=minimumPivot);
        if not poseConsistent then
          result.sequentialDiagnostics.verified := false;
          result.sequentialDiagnostics.rejectionReason := 11;
        end if;
      end if;
    end if;
    (captured,stored,slot,evicted) := RGBDKeyframes.Store(previousCatalog,result.loopDiagnostics.prepared,result.loopDiagnostics.retrievalAccepted);
    result.graphDiagnostics := RGBDGraphMeasurements.Capture(previousCatalog,captured,previousGraph,
      result.sequentialDiagnostics,result.loopDiagnostics.proposals,stored,false,inlierDistance,maximumRms,
      calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared,
      localizationSigma=localizationSigma,depthInflation=depthInflation,minimumPivot=minimumPivot,
      loopInlierDistance=loopInlierDistance, loopMaximumRms=loopMaximumRms, loopCalibratedResiduals=loopCalibratedResiduals, loopMaximumNormalizedSquared=loopMaximumNormalizedSquared);
    result.problem := RGBDGraphMeasurements.EmptyProblem();
    if result.graphDiagnostics.accepted then
      result.problem := RGBDGraphMeasurements.PrepareProblem(captured,result.graphDiagnostics.state,true);
    end if;
    if requested then
      result.rejectionReason := if not result.loopDiagnostics.retrievalAccepted then 2 else if not stored then 3
        else if not result.graphDiagnostics.accepted then 4 else if not result.problem.accepted then 5 else 0;
      if result.rejectionReason == 0 then
        result.catalog := captured; result.graph := result.graphDiagnostics.state;
        result.accepted := true; result.storedSlot := slot; result.evictedId := evicted;
      end if;
    end if;
  end Capture;
end RGBDCatalogGraphCapture;
