within SLAM.Mapping;
// Corrected pose metadata is explicit; immutable raw catalog poses stay raw.
// This package does not import RGBDGraphEstimatorCommit (no cyclic dependency).
package RGBDCatalogPoseMapping
  import RGBDCatalogFrame = SLAM.Mapping.RGBDCatalogFrame;
  import RGBDCatalogMapping = SLAM.Mapping.RGBDCatalogMapping;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframePolicy = SLAM.LoopClosure.RGBDKeyframePolicy;
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;

  constant Integer nodeCapacity = RGBDKeyframes.keyframeCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;
  record PoseView
    Integer generation; Integer sourceRevision; Integer revision; Integer catalogNextId;
    Boolean enabled[nodeCapacity]; Integer ids[nodeCapacity]; Real positions[nodeCapacity,dimension]; Real rotations[nodeCapacity,dimension,dimension];
  end PoseView;
  record Result
    RGBDCatalogMapping.Result mapping; PoseView poses; RGBDKeyframePolicy.Decision decision;
    Boolean accepted; Integer rejectionReason;
  end Result;
  function FromCatalog
    input RGBDKeyframes.Catalog catalog; input Integer sourceRevision; output PoseView poses;
  algorithm
    poses.generation := catalog.generation; poses.sourceRevision := sourceRevision; poses.revision := 0;
    poses.catalogNextId := catalog.nextId; poses.enabled := catalog.occupied; poses.ids := catalog.ids;
    poses.positions := catalog.bodyPositions; poses.rotations := catalog.bodyRotations;
  end FromCatalog;
  function ValidView
    input RGBDKeyframes.Catalog catalog; input PoseView poses; input Integer sourceRevision; output Boolean valid;
  algorithm
    valid := RGBDKeyframes.ValidHeader(catalog) and poses.generation == catalog.generation
      and sourceRevision > 0 and sourceRevision <= RGBDKeyframes.identifierLimit and poses.sourceRevision == sourceRevision
      and poses.revision >= 0 and poses.revision < RGBDKeyframes.identifierLimit-1 and poses.catalogNextId == catalog.nextId;
    for slot in 1:nodeCapacity loop
      valid := valid and poses.enabled[slot] == catalog.occupied[slot];
      if poses.enabled[slot] then
        valid := valid and poses.ids[slot] == catalog.ids[slot] and RGBDUncertaintyProper(poses.rotations[slot,:,:]);
        for axis in 1:dimension loop valid := valid and abs(poses.positions[slot,axis]) <= 1e6; end for;
      end if;
    end for;
  end ValidView;
  function Advance
    input RGBDKeyframes.Catalog catalog;
    input RGBDGraphMeasurements.State graph;
    input RGBDCatalogMapping.State map;
    input PoseView poses; input Integer sourceRevision;
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
    output Result result;
  protected Integer slot;
  algorithm
    result.poses := poses; result.accepted := false; result.rejectionReason := 1;
    result.mapping.catalog := catalog; result.mapping.graph := graph; result.mapping.map := map;
    result.mapping.accepted := false; result.mapping.rejectionReason := 1;
    result.mapping.diagnostics := RGBDCatalogMapping.EmptyDiagnostics(); result.mapping.problem := RGBDGraphMeasurements.EmptyProblem();
    result.decision.valid := false; result.decision.captureRequested := false; result.decision.reason := 1;
    result.decision.enabledCount := 0; result.decision.elapsed := 0; result.decision.translationSquared := 0; result.decision.rotationCosine := 1;
    if requested then
      result.rejectionReason := 2;
      if ValidView(catalog,poses,sourceRevision) then
        (result.mapping,result.decision) := RGBDCatalogFrame.Advance(
          catalog,graph,map,measurement,vocabulary,vocabularyEnabled,candidatePoint,candidateEnabled,imageFresh,poseAccepted,requested,minimumInterval,maximumInterval,translationThreshold,rotationThreshold,minimumFeatures,seeds,sequentialSeed,maximumWordDistanceSquared,minimumAssignments,minimumSimilarity,minimumAge,trials,refinements,minimumInliers,minimumFraction,inlierDistance,maximumRms,descriptorRatio,maximumDescriptorDistance,registrationCoordinateLimit,rankTolerance,localizationSigma,depthInflation,minimumPivot,coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,confirmationObservations,maximumConfidence,maximumTentative,consistencyTolerance,poses.positions,poses.rotations,poses.revision);
        result.rejectionReason := 3;
        if result.mapping.accepted then
          if result.mapping.catalog.nextId == catalog.nextId+1 then
            slot := catalog.nextSlot;
            result.poses.revision := poses.revision+1; result.poses.catalogNextId := result.mapping.catalog.nextId;
            result.poses.enabled[slot] := true; result.poses.ids[slot] := result.mapping.catalog.ids[slot];
            result.poses.positions[slot,:] := result.mapping.catalog.bodyPositions[slot,:];
            result.poses.rotations[slot,:,:] := result.mapping.catalog.bodyRotations[slot,:,:];
          end if;
          result.accepted := true; result.rejectionReason := 0;
        end if;
      end if;
    end if;
  end Advance;
end RGBDCatalogPoseMapping;
