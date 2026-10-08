within SLAM.Mapping;
// Mapping between captures: the retained catalog and raw measured graph hold.
// Reuses the capture/map owner's noncapture validation and insertion receipts.
package RGBDCatalogObservation
  import RGBDCatalogMapping = SLAM.Mapping.RGBDCatalogMapping;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import UpdateKeyframeLandmarks = SLAM.Mapping.UpdateKeyframeLandmarks;

  function Update
    input RGBDKeyframes.Catalog previousCatalog;
    input RGBDGraphMeasurements.State previousGraph;
    input RGBDCatalogMapping.State previous;
    input RGBDKeyframes.Frame measurement;
    input Real candidatePoint[RGBDKeyframes.featureCapacity,RGBDKeyframes.dimension];
    input Real candidateEnabled[RGBDKeyframes.featureCapacity];
    input Real poseAccepted;
    input Boolean requested;
    input Real coordinateLimit = 1e6; input Real voxelWidth = 0.25;
    input Real mergeRadius = 0.15; input Real maximumDistance = 80.0;
    input Real tentativeLifetime = 0.5; input Real confirmedLifetime = 5.0;
    input Real confirmationObservations = 3.0; input Real maximumConfidence = 8.0;
    input Real maximumTentative = 700.0; input Real consistencyTolerance = 1e-6;
    input Real nodePosition[RGBDKeyframes.keyframeCapacity,3] = previousCatalog.bodyPositions;
    input Real nodeRotation[RGBDKeyframes.keyframeCapacity,3,3] = previousCatalog.bodyRotations;
    input Integer poseRevision = previous.catalogRevision;
    output RGBDCatalogMapping.Result result;
  protected
    RGBDCatalogMapping.State proposed;
    RGBDCatalogMapping.Diagnostics diagnostics;
    RGBDKeyframes.Catalog unchangedCatalog;
    Real confirmed[RGBDCatalogMapping.mapCapacity]; Real updateAccepted; Real updateReason;
    Boolean valid; Boolean captured; Integer storedSlot; Integer evictedId; Integer measurementReason;
  algorithm
    result.catalog := previousCatalog; result.graph := previousGraph; result.map := previous;
    result.accepted := false; result.rejectionReason := 1;
    result.diagnostics := RGBDCatalogMapping.EmptyDiagnostics();
    result.problem := RGBDGraphMeasurements.EmptyProblem();
    if requested then
      result.rejectionReason := 3;
      valid := RGBDKeyframes.ValidHeader(previousCatalog)
        and previous.generation == previousCatalog.generation
        and previousGraph.generation == previousCatalog.generation
        and previousGraph.revision >= 0 and previousGraph.revision <= RGBDKeyframes.identifierLimit
        and previous.catalogRevision >= 0 and previous.catalogRevision <= RGBDKeyframes.identifierLimit
        and previous.frame >= 0 and previous.frame < RGBDKeyframes.identifierLimit
        and floor(previous.frame) == previous.frame
        and previous.imageEpoch >= -1 and previous.imageEpoch <= RGBDKeyframes.identifierLimit
        and ((previous.frame == 0 and previous.imageEpoch == -1)
          or (previous.frame > 0 and previous.imageEpoch >= 0))
        and measurement.id == previousCatalog.nextId
        and measurement.epoch > previous.imageEpoch
        and measurement.epoch > previousCatalog.lastEpoch
        and measurement.imageTime >= previousCatalog.lastTime
        and poseAccepted == 1.0;
      if valid then
        valid := previousGraph.lastCaptureId == previousCatalog.nextId-1;
      end if;
      if valid then
        proposed := previous; diagnostics := RGBDCatalogMapping.EmptyDiagnostics();
        (proposed.point,proposed.occupied,proposed.confidence,proposed.lastSeen,proposed.lastFrame,confirmed,
          updateAccepted,updateReason,proposed.imageTime,proposed.frame,proposed.worldFrame,
          diagnostics.occupiedCount,diagnostics.confirmedCount,diagnostics.tentativeCount,
          diagnostics.insertedCount,diagnostics.mergedCount,diagnostics.prunedCount,diagnostics.droppedCount,diagnostics.invalidCandidateCount,
          unchangedCatalog,proposed.localPoint,proposed.anchorId,proposed.anchorSlot,proposed.generation,proposed.catalogRevision,
          captured,storedSlot,evictedId,measurementReason,diagnostics.catalogRejectionReason,diagnostics.correctionReason,
          diagnostics.catalogUpdateRejectionReason,diagnostics.updateRejectionReason,
          diagnostics.mapRejectionReason,diagnostics.anchorRejectionReason,diagnostics.projectedCount,
          diagnostics.evictedCount,diagnostics.assignedCount,diagnostics.retainedCount,diagnostics.clearedCount) := UpdateKeyframeLandmarks(
            previous.point,previous.occupied,previous.confidence,previous.lastSeen,previous.lastFrame,
            candidatePoint,candidateEnabled,measurement.count,measurement.bodyPosition,poseAccepted,
            previous.imageTime,measurement.imageTime,previous.frame,previous.frame+1,previous.worldFrame,previous.worldFrame,0.0,
            coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,
            confirmationObservations,maximumConfidence,maximumTentative,
            previousCatalog,measurement,measurement.epoch,previousCatalog.generation,false,true,
            previous.localPoint,previous.anchorId,previous.anchorSlot,previous.generation,previous.catalogRevision,consistencyTolerance,
            nodePosition,nodeRotation,poseRevision);
        diagnostics.mapAccepted := updateAccepted;
        result.diagnostics := diagnostics;
        result.rejectionReason := if measurementReason <> 0 then 4 else 5;
        if updateAccepted == 1.0 and not captured and storedSlot == 0 and evictedId == 0 then
          proposed.imageEpoch := measurement.epoch;
          result.map := proposed; result.accepted := true; result.rejectionReason := 0;
        end if;
      end if;
    end if;
  end Update;
end RGBDCatalogObservation;
