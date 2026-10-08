within SLAM.Mapping;
// Capture of a descriptor/calibration/pose/histogram record joins the map
// transaction. The enclosing estimator/reference/graph must join this commit.
function UpdateKeyframeLandmarks
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLandmarkMapInterface = SLAM.Mapping.RGBDLandmarkMapInterface;
  import RGBDMapAnchors = SLAM.Mapping.RGBDMapAnchors;
  import UpdateCatalogLandmarkMap = SLAM.Mapping.UpdateCatalogLandmarkMap;

  extends RGBDLandmarkMapInterface;
  input RGBDKeyframes.Catalog previousCatalog;
  input RGBDKeyframes.Frame measurement;
  input Integer imageEpoch; input Integer generation;
  input Boolean captureRequested; input Boolean requested;
  input Real previousLocalPoint[size(previousOccupied,1),3];
  input Integer previousAnchorId[size(previousOccupied,1)]; input Integer previousAnchorSlot[size(previousOccupied,1)];
  input Integer previousGeneration; input Integer previousCatalogRevision;
  input Real consistencyTolerance;
  input Real previousNodePosition[RGBDKeyframes.keyframeCapacity,3] = previousCatalog.bodyPositions;
  input Real previousNodeRotation[RGBDKeyframes.keyframeCapacity,3,3] = previousCatalog.bodyRotations;
  input Integer previousPoseRevision = previousCatalogRevision;
  output RGBDKeyframes.Catalog nextCatalog;
  output Real localPoint[size(previousOccupied,1),3];
  output Integer anchorId[size(previousOccupied,1)]; output Integer anchorSlot[size(previousOccupied,1)];
  output Integer nextGeneration; output Integer nextCatalogRevision;
  output Boolean captureAccepted; output Integer storedSlot; output Integer evictedId;
  output Integer measurementRejectionReason; output Integer catalogRejectionReason; output Integer correctionReason;
  output Real catalogUpdateRejectionReason; output Real updateRejectionReason;
  output Real mapRejectionReason; output Integer anchorRejectionReason;
  output Integer projectedCount; output Integer evictedCount;
  output Integer assignedCount; output Integer retainedCount; output Integer clearedCount;
protected
  RGBDKeyframes.Catalog baseCatalog; RGBDKeyframes.Catalog proposedCatalog;
  Boolean configuration; Boolean valid; Boolean featureValid; Boolean resetAccepted;
  Boolean stored; Integer proposedSlot; Integer proposedEvicted; Integer revision;
  Integer selectedId; Integer selectedSlot; Real expectedWorld[3];
  Real proposedNodePosition[RGBDKeyframes.keyframeCapacity,3];
  Real proposedNodeRotation[RGBDKeyframes.keyframeCapacity,3,3]; Integer poseRevision;
algorithm
  point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
  lastSeen := previousLastSeen; lastFrame := previousLastFrame;
  localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
  nextCatalog := previousCatalog; nextGeneration := previousGeneration; nextCatalogRevision := previousCatalogRevision;
  nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
  accepted := 0; rejectionReason := 1; measurementRejectionReason := 0; catalogRejectionReason := 0; correctionReason := 0;
  catalogUpdateRejectionReason := 0; updateRejectionReason := 0; mapRejectionReason := 0; anchorRejectionReason := 0;
  captureAccepted := false; storedSlot := 0; evictedId := 0;
  projectedCount := 0; evictedCount := 0; assignedCount := 0; retainedCount := 0; clearedCount := 0;
  insertedCount := 0; mergedCount := 0; prunedCount := 0; droppedCount := 0; invalidCandidateCount := 0;
  if requested then
    configuration := previousGeneration == previousCatalog.generation
      and generation >= 1 and generation <= RGBDKeyframes.identifierLimit
      and previousGeneration >= 1 and previousGeneration <= RGBDKeyframes.identifierLimit
      and previousCatalogRevision >= 0 and previousCatalogRevision <= RGBDKeyframes.identifierLimit
      and ((resetRequested == 1.0 and previousGeneration < RGBDKeyframes.identifierLimit and generation == previousGeneration+1)
        or (resetRequested == 0.0 and generation == previousGeneration))
      and size(candidatePoint,1) == RGBDKeyframes.featureCapacity and size(candidatePoint,2) == RGBDKeyframes.dimension
      and size(candidateEnabled,1) == RGBDKeyframes.featureCapacity
      and imageEpoch >= 0 and imageEpoch <= RGBDKeyframes.identifierLimit
      and consistencyTolerance > 0 and consistencyTolerance <= 0.01
      and coordinateLimit > 0 and coordinateLimit <= 1e6;
    rejectionReason := 2; measurementRejectionReason := 1;
    if configuration then
      baseCatalog := previousCatalog; resetAccepted := true;
      if resetRequested == 1.0 then
        (baseCatalog,resetAccepted) := RGBDKeyframes.Reset(previousCatalog,generation,measurement.vocabularyVersion);
      end if;
      valid := resetAccepted and RGBDKeyframes.ValidHeader(baseCatalog)
        and measurement.generation == generation and measurement.vocabularyVersion == baseCatalog.vocabularyVersion
        and measurement.epoch == imageEpoch and measurement.imageTime == timeNow
        and measurement.count >= 0 and measurement.count <= RGBDKeyframes.featureCapacity
        and candidateCount == measurement.count
        and RGBDMapAnchors.ProperRotation(measurement.bodyRotation)
        and RGBDMapAnchors.ProperRotation(measurement.opticalToBody);
      for axis in 1:RGBDKeyframes.dimension loop
        valid := valid and bodyPosition[axis] == measurement.bodyPosition[axis]
          and abs(measurement.bodyPosition[axis]) <= coordinateLimit
          and abs(measurement.cameraOriginBody[axis]) <= coordinateLimit;
      end for;
      // Check only enabled geometry; descriptors and calibration are admitted
      // by Store when capture is requested. Noncapture frames may have fewer
      // than eight features, including a zero-feature observation.
      if valid then
        for feature in 1:RGBDKeyframes.featureCapacity loop
          featureValid := candidateEnabled[feature] == 0.0 or candidateEnabled[feature] == 1.0;
          if candidateEnabled[feature] == 1.0 then
            featureValid := measurement.enabled[feature] and feature <= measurement.count
              and measurement.opticalPoint[feature,3] > 0;
            for axis in 1:RGBDKeyframes.dimension loop
              featureValid := featureValid and abs(measurement.opticalPoint[feature,axis]) <= coordinateLimit;
            end for;
            if featureValid then
              expectedWorld := measurement.bodyRotation*(measurement.opticalToBody*measurement.opticalPoint[feature,:]
                +measurement.cameraOriginBody)+measurement.bodyPosition;
              for axis in 1:RGBDKeyframes.dimension loop
                featureValid := featureValid and abs(expectedWorld[axis]) <= coordinateLimit
                  and abs(candidatePoint[feature,axis]-expectedWorld[axis]) <= consistencyTolerance;
              end for;
            end if;
          end if;
          valid := valid and featureValid;
        end for;
      end if;
      measurementRejectionReason := 2;
      if valid then
        proposedCatalog := baseCatalog; stored := false; proposedSlot := 0; proposedEvicted := 0;
        if captureRequested then
          (proposedCatalog,stored,proposedSlot,proposedEvicted) := RGBDKeyframes.Store(baseCatalog,measurement,true);
          valid := stored;
        end if;
        measurementRejectionReason := 3;
        if valid then
          revision := if resetRequested == 1.0 then 0 else previousCatalogRevision+(if stored then 1 else 0);
          proposedNodePosition := previousNodePosition; proposedNodeRotation := previousNodeRotation;
          if stored then
            proposedNodePosition[proposedSlot,:] := measurement.bodyPosition;
            proposedNodeRotation[proposedSlot,:,:] := measurement.bodyRotation;
          end if;
          poseRevision := if resetRequested == 1.0 then 0 else previousPoseRevision+(if stored then 1 else 0);
          selectedId := 0; selectedSlot := 0;
          if proposedCatalog.nextId > 1 then
            selectedSlot := mod(proposedCatalog.nextId-2,RGBDKeyframes.keyframeCapacity)+1;
            selectedId := proposedCatalog.ids[selectedSlot];
          end if;
          (point,occupied,confidence,lastSeen,lastFrame,confirmed,accepted,catalogUpdateRejectionReason,nextTime,nextFrame,nextWorldFrame,
            occupiedCount,confirmedCount,tentativeCount,insertedCount,mergedCount,prunedCount,droppedCount,invalidCandidateCount,
            localPoint,anchorId,anchorSlot,nextGeneration,nextCatalogRevision,catalogRejectionReason,correctionReason,
            updateRejectionReason,mapRejectionReason,anchorRejectionReason,projectedCount,evictedCount,
            assignedCount,retainedCount,clearedCount) := UpdateCatalogLandmarkMap(
              previousPoint,previousOccupied,previousConfidence,previousLastSeen,previousLastFrame,candidatePoint,candidateEnabled,
              candidateCount,bodyPosition,poseAccepted,previousTime,timeNow,previousFrame,frameNow,previousWorldFrame,worldFrame,resetRequested,
              coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,confirmationObservations,
              maximumConfidence,maximumTentative,previousLocalPoint,previousAnchorId,previousAnchorSlot,previousGeneration,
              previousCatalogRevision,previousCatalog.occupied,previousCatalog.ids,previousNodePosition,previousNodeRotation,
              proposedCatalog.occupied,proposedCatalog.ids,proposedNodePosition,proposedNodeRotation,
              generation,revision,selectedId,selectedSlot,true,true,consistencyTolerance,previousPoseRevision,poseRevision);
          rejectionReason := if accepted == 1.0 then 0 else 3;
          measurementRejectionReason := 0;
          if accepted == 1.0 then
            nextCatalog := proposedCatalog; captureAccepted := stored;
            storedSlot := proposedSlot; evictedId := proposedEvicted;
          end if;
        end if;
      end if;
    end if;
  end if;
  if accepted <> 1.0 then
    nextCatalog := previousCatalog; captureAccepted := false; storedSlot := 0; evictedId := 0;
    point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
    lastSeen := previousLastSeen; lastFrame := previousLastFrame;
    localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
    nextGeneration := previousGeneration; nextCatalogRevision := previousCatalogRevision;
    nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
    projectedCount := 0; evictedCount := 0; assignedCount := 0; retainedCount := 0; clearedCount := 0;
    insertedCount := 0; mergedCount := 0; prunedCount := 0; droppedCount := 0; invalidCandidateCount := 0;
    occupiedCount := 0; confirmedCount := 0; tentativeCount := 0;
    for slot in 1:size(previousOccupied,1) loop
      confirmed[slot] := if occupied[slot] == 1.0 and confidence[slot] >= confirmationObservations then 1.0 else 0.0;
      occupiedCount := occupiedCount+(if occupied[slot] == 1.0 then 1.0 else 0.0);
      confirmedCount := confirmedCount+confirmed[slot];
      tentativeCount := tentativeCount+(if occupied[slot] == 1.0 and confirmed[slot] == 0.0 then 1.0 else 0.0);
    end for;
  end if;
end UpdateKeyframeLandmarks;
