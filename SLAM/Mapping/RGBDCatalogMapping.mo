within SLAM.Mapping;
// Mapping consumes an admitted visual/catalog/graph proposal. Publication is
// still local until estimator/reference and any graph correction also accept.
package RGBDCatalogMapping
  import RGBDCatalogGraphCapture = SLAM.PoseGraph.RGBDCatalogGraphCapture;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDMapAnchors = SLAM.Mapping.RGBDMapAnchors;
  import UpdateCatalogLandmarkMap = SLAM.Mapping.UpdateCatalogLandmarkMap;

  constant Integer mapCapacity = RGBDMapAnchors.mapCapacity;
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;

  record State
    Real point[mapCapacity,dimension]; Real occupied[mapCapacity];
    Real confidence[mapCapacity]; Real lastSeen[mapCapacity]; Real lastFrame[mapCapacity];
    Real localPoint[mapCapacity,dimension];
    Integer anchorId[mapCapacity]; Integer anchorSlot[mapCapacity];
    Integer generation; Integer catalogRevision;
    Real imageTime; Integer imageEpoch;
    Real frame "Consecutive accepted map-observation counter, not camera epoch";
    Real worldFrame;
  end State;

  function Empty
    input Integer generation = 1;
    input Real worldFrame = 0.0;
    output State result;
  algorithm
    result.point := zeros(mapCapacity,dimension); result.occupied := zeros(mapCapacity);
    result.confidence := zeros(mapCapacity); result.lastSeen := zeros(mapCapacity); result.lastFrame := zeros(mapCapacity);
    result.localPoint := zeros(mapCapacity,dimension);
    result.anchorId := fill(0,mapCapacity); result.anchorSlot := fill(0,mapCapacity);
    result.generation := generation; result.catalogRevision := 0;
    result.imageTime := 0.0; result.imageEpoch := -1; result.frame := 0.0; result.worldFrame := worldFrame;
  end Empty;

  record Diagnostics
    Real mapAccepted; Real catalogUpdateRejectionReason;
    Integer keyframeRejectionReason; Integer sequentialRejectionReason;
    Integer catalogRejectionReason; Integer correctionReason;
    Real updateRejectionReason; Real mapRejectionReason; Integer anchorRejectionReason;
    Real occupiedCount; Real confirmedCount; Real tentativeCount;
    Real insertedCount; Real mergedCount; Real prunedCount; Real droppedCount; Real invalidCandidateCount;
    Integer projectedCount; Integer evictedCount; Integer assignedCount; Integer retainedCount; Integer clearedCount;
  end Diagnostics;

  function EmptyDiagnostics
    output Diagnostics result;
  algorithm
    result.mapAccepted := 0.0; result.catalogUpdateRejectionReason := 0.0;
    result.keyframeRejectionReason := 0; result.sequentialRejectionReason := 0;
    result.catalogRejectionReason := 0; result.correctionReason := 0;
    result.updateRejectionReason := 0.0; result.mapRejectionReason := 0.0; result.anchorRejectionReason := 0;
    result.occupiedCount := 0.0; result.confirmedCount := 0.0; result.tentativeCount := 0.0;
    result.insertedCount := 0.0; result.mergedCount := 0.0; result.prunedCount := 0.0;
    result.droppedCount := 0.0; result.invalidCandidateCount := 0.0;
    result.projectedCount := 0; result.evictedCount := 0; result.assignedCount := 0;
    result.retainedCount := 0; result.clearedCount := 0;
  end EmptyDiagnostics;

  record Result
    RGBDKeyframes.Catalog catalog;
    RGBDGraphMeasurements.State graph;
    State map;
    Boolean accepted;
    Integer rejectionReason "1 idle; 2 visual; 3 binding; 4 projection; 5 map";
    Diagnostics diagnostics "Map-stage diagnostics are proposals, not committed counts";
    RGBDGraphMeasurements.Problem problem;
  end Result;

  function Capture
    input RGBDKeyframes.Catalog previousCatalog;
    input RGBDGraphMeasurements.State previousGraph;
    input State previous;
    input RGBDCatalogGraphCapture.Result visual "From the same old catalog/graph owners";
    input Real candidatePoint[featureCapacity,dimension];
    input Real candidateEnabled[featureCapacity];
    input Real poseAccepted;
    input Boolean requested;
    input Real coordinateLimit = 1e6; input Real voxelWidth = 0.25;
    input Real mergeRadius = 0.15; input Real maximumDistance = 80.0;
    input Real tentativeLifetime = 0.5; input Real confirmedLifetime = 5.0;
    input Real confirmationObservations = 3.0; input Real maximumConfidence = 8.0;
    input Real maximumTentative = 700.0; input Real consistencyTolerance = 1e-6;
    input Real previousNodePosition[RGBDKeyframes.keyframeCapacity,3] = previousCatalog.bodyPositions;
    input Real previousNodeRotation[RGBDKeyframes.keyframeCapacity,3,3] = previousCatalog.bodyRotations;
    input Integer previousPoseRevision = previous.catalogRevision;
    output Result result;
  protected
    State proposed;
    RGBDKeyframes.Frame measurement;
    Diagnostics diagnostics;
    Real confirmed[mapCapacity];
    Integer slot; Boolean valid; Boolean candidateValid; Real expectedWorld[dimension];
    Real nodePosition[RGBDKeyframes.keyframeCapacity,3]; Real nodeRotation[RGBDKeyframes.keyframeCapacity,3,3];
  algorithm
    result.catalog := previousCatalog; result.graph := previousGraph; result.map := previous;
    result.accepted := false; result.rejectionReason := 1;
    result.diagnostics := EmptyDiagnostics(); result.problem := RGBDGraphMeasurements.EmptyProblem();
    if requested then
      result.rejectionReason := 2;
      if visual.accepted then
        result.rejectionReason := 3;
        valid := RGBDKeyframes.ValidHeader(previousCatalog) and RGBDKeyframes.ValidHeader(visual.catalog)
          and previous.generation == previousCatalog.generation
          and visual.catalog.generation == previousCatalog.generation
          and previousCatalog.nextId < RGBDKeyframes.identifierLimit
          and visual.catalog.nextId >= 2
          and previous.catalogRevision >= 0 and previous.catalogRevision < RGBDKeyframes.identifierLimit
          and previous.frame >= 0 and previous.frame < RGBDKeyframes.identifierLimit
          and floor(previous.frame) == previous.frame
          and previous.imageEpoch >= -1 and previous.imageEpoch <= RGBDKeyframes.identifierLimit
          and ((previous.frame == 0 and previous.imageEpoch == -1)
            or (previous.frame > 0 and previous.imageEpoch >= 0))
          and visual.graph.generation == previousGraph.generation
          and previousGraph.revision >= 0 and previousGraph.revision < RGBDKeyframes.identifierLimit
          and coordinateLimit > 0 and coordinateLimit <= 1e6
          and consistencyTolerance > 0 and consistencyTolerance <= 0.01
          and visual.storedSlot >= 1 and visual.storedSlot <= RGBDKeyframes.keyframeCapacity
          and visual.problem.accepted;
        if valid then
          valid := visual.catalog.nextId == previousCatalog.nextId+1
            and visual.graph.revision == previousGraph.revision+1
            and visual.graph.lastCaptureId == visual.catalog.nextId-1
            and visual.storedSlot == previousCatalog.nextSlot;
          if valid then
            slot := visual.storedSlot;
            measurement := RGBDKeyframes.ReadSlot(visual.catalog,slot);
            valid := measurement.id == previousCatalog.nextId
              and measurement.epoch == visual.catalog.lastEpoch
              and measurement.epoch > previous.imageEpoch
              and measurement.imageTime == visual.catalog.lastTime
              and RGBDKeyframes.ValidFrame(measurement)
              and poseAccepted == 1.0;
            result.rejectionReason := 4;
            if valid then
              for feature in 1:featureCapacity loop
                candidateValid := candidateEnabled[feature] == 0.0 or candidateEnabled[feature] == 1.0;
                if candidateEnabled[feature] == 1.0 then
                  candidateValid := measurement.enabled[feature] and feature <= measurement.count;
                  if candidateValid then
                    expectedWorld := measurement.bodyRotation*(measurement.opticalToBody*measurement.opticalPoint[feature,:]
                      +measurement.cameraOriginBody)+measurement.bodyPosition;
                    for axis in 1:dimension loop
                      candidateValid := candidateValid and abs(expectedWorld[axis]) <= coordinateLimit
                        and abs(candidatePoint[feature,axis]-expectedWorld[axis]) <= consistencyTolerance;
                    end for;
                  end if;
                end if;
                valid := valid and candidateValid;
              end for;
            end if;
            if valid then
              proposed := previous; diagnostics := EmptyDiagnostics();
              nodePosition := previousNodePosition; nodeRotation := previousNodeRotation;
              nodePosition[slot,:] := measurement.bodyPosition; nodeRotation[slot,:,:] := measurement.bodyRotation;
              (proposed.point,proposed.occupied,proposed.confidence,proposed.lastSeen,proposed.lastFrame,confirmed,
                diagnostics.mapAccepted,diagnostics.catalogUpdateRejectionReason,proposed.imageTime,proposed.frame,proposed.worldFrame,
                diagnostics.occupiedCount,diagnostics.confirmedCount,diagnostics.tentativeCount,
                diagnostics.insertedCount,diagnostics.mergedCount,diagnostics.prunedCount,diagnostics.droppedCount,diagnostics.invalidCandidateCount,
                proposed.localPoint,proposed.anchorId,proposed.anchorSlot,proposed.generation,proposed.catalogRevision,
                diagnostics.catalogRejectionReason,diagnostics.correctionReason,diagnostics.updateRejectionReason,
                diagnostics.mapRejectionReason,diagnostics.anchorRejectionReason,diagnostics.projectedCount,
                diagnostics.evictedCount,diagnostics.assignedCount,diagnostics.retainedCount,diagnostics.clearedCount) := UpdateCatalogLandmarkMap(
                  previous.point,previous.occupied,previous.confidence,previous.lastSeen,previous.lastFrame,
                  candidatePoint,candidateEnabled,measurement.count,measurement.bodyPosition,poseAccepted,
                  previous.imageTime,measurement.imageTime,previous.frame,previous.frame+1,previous.worldFrame,previous.worldFrame,0.0,
                  coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,
                  confirmationObservations,maximumConfidence,maximumTentative,
                  previous.localPoint,previous.anchorId,previous.anchorSlot,previous.generation,previous.catalogRevision,
                  previousCatalog.occupied,previousCatalog.ids,previousNodePosition,previousNodeRotation,
                  visual.catalog.occupied,visual.catalog.ids,nodePosition,nodeRotation,
                  visual.catalog.generation,previous.catalogRevision+1,measurement.id,slot,true,true,consistencyTolerance,
                  previousPoseRevision,previousPoseRevision+1);
              result.diagnostics := diagnostics; result.rejectionReason := 5;
              if diagnostics.mapAccepted == 1.0 then
                proposed.imageEpoch := measurement.epoch;
                result.catalog := visual.catalog; result.graph := visual.graph; result.map := proposed;
                result.problem := visual.problem; result.accepted := true; result.rejectionReason := 0;
              end if;
            end if;
          end if;
        end if;
      end if;
    end if;
  end Capture;
end RGBDCatalogMapping;
