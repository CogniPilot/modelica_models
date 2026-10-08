within SLAM.Mapping;
function UpdateCatalogLandmarkMap
  import RGBDLandmarkCatalog = SLAM.Mapping.RGBDLandmarkCatalog;
  import RGBDLandmarkMapInterface = SLAM.Mapping.RGBDLandmarkMapInterface;
  import UpdateAnchoredLandmarkMap = SLAM.Mapping.UpdateAnchoredLandmarkMap;

  extends RGBDLandmarkMapInterface;
  input Real previousLocalPoint[size(previousOccupied,1),3];
  input Integer previousAnchorId[size(previousOccupied,1)]; input Integer previousAnchorSlot[size(previousOccupied,1)];
  input Integer previousGeneration; input Integer previousCatalogRevision;
  input Boolean previousNodeEnabled[:]; input Integer previousNodeId[size(previousNodeEnabled,1)];
  input Real previousNodePosition[size(previousNodeEnabled,1),3];
  input Real previousNodeRotation[size(previousNodeEnabled,1),3,3];
  input Boolean nodeEnabled[size(previousNodeEnabled,1)]; input Integer nodeId[size(previousNodeEnabled,1)];
  input Real nodePosition[size(previousNodeEnabled,1),3]; input Real nodeRotation[size(previousNodeEnabled,1),3,3];
  input Integer catalogGeneration; input Integer catalogRevision;
  input Integer selectedAnchorId; input Integer selectedAnchorSlot;
  input Boolean catalogAccepted; input Boolean requested; input Real consistencyTolerance;
  input Integer previousPoseRevision = previousCatalogRevision;
  input Integer poseRevision = catalogRevision;
  output Real localPoint[size(previousOccupied,1),3];
  output Integer anchorId[size(previousOccupied,1)]; output Integer anchorSlot[size(previousOccupied,1)];
  output Integer nextGeneration; output Integer nextCatalogRevision;
  output Integer catalogRejectionReason; output Integer correctionReason;
  output Real updateRejectionReason; output Real mapRejectionReason; output Integer anchorRejectionReason;
  output Integer projectedCount; output Integer evictedCount;
  output Integer assignedCount; output Integer retainedCount; output Integer clearedCount;
protected
  Real preparedPoint[size(previousOccupied,1),3]; Real preparedOccupied[size(previousOccupied,1)];
  Real preparedConfidence[size(previousOccupied,1)]; Real preparedLastSeen[size(previousOccupied,1)];
  Real preparedLastFrame[size(previousOccupied,1)]; Real preparedLocal[size(previousOccupied,1),3];
  Integer preparedId[size(previousOccupied,1)]; Integer preparedSlot[size(previousOccupied,1)]; Boolean prepared;
algorithm
  point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
  lastSeen := previousLastSeen; lastFrame := previousLastFrame;
  localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
  nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
  nextGeneration := previousGeneration; nextCatalogRevision := previousCatalogRevision;
  accepted := 0; rejectionReason := 1; updateRejectionReason := 0; mapRejectionReason := 0; anchorRejectionReason := 0;
  projectedCount := 0; evictedCount := 0; assignedCount := 0; retainedCount := 0; clearedCount := 0;
  insertedCount := 0; mergedCount := 0; prunedCount := 0; droppedCount := 0; invalidCandidateCount := 0;
  (preparedPoint,preparedOccupied,preparedConfidence,preparedLastSeen,preparedLastFrame,
    preparedLocal,preparedId,preparedSlot,prepared,catalogRejectionReason,correctionReason,projectedCount,evictedCount) :=
    RGBDLandmarkCatalog.Synchronize(previousPoint,previousOccupied,previousConfidence,previousLastSeen,previousLastFrame,
      previousLocalPoint,previousAnchorId,previousAnchorSlot,previousGeneration,previousPoseRevision,
      previousNodeEnabled,previousNodeId,previousNodePosition,previousNodeRotation,
      nodeEnabled,nodeId,nodePosition,nodeRotation,catalogGeneration,poseRevision,catalogAccepted,requested,
      resetRequested == 1.0,previousTime,previousFrame,maximumConfidence,coordinateLimit,consistencyTolerance);
  if requested then
    rejectionReason := 2;
    if prepared then
      (point,occupied,confidence,lastSeen,lastFrame,confirmed,accepted,updateRejectionReason,nextTime,nextFrame,nextWorldFrame,
        occupiedCount,confirmedCount,tentativeCount,insertedCount,mergedCount,prunedCount,droppedCount,invalidCandidateCount,
        localPoint,anchorId,anchorSlot,nextGeneration,mapRejectionReason,anchorRejectionReason,assignedCount,retainedCount,clearedCount) :=
        UpdateAnchoredLandmarkMap(preparedPoint,preparedOccupied,preparedConfidence,preparedLastSeen,preparedLastFrame,
          candidatePoint,candidateEnabled,candidateCount,bodyPosition,poseAccepted,previousTime,timeNow,previousFrame,frameNow,
          previousWorldFrame,worldFrame,resetRequested,coordinateLimit,voxelWidth,mergeRadius,maximumDistance,
          tentativeLifetime,confirmedLifetime,confirmationObservations,maximumConfidence,maximumTentative,
          preparedLocal,preparedId,preparedSlot,previousGeneration,nodeEnabled,nodeId,nodePosition,nodeRotation,
          catalogGeneration,catalogGeneration,selectedAnchorId,selectedAnchorSlot,true,consistencyTolerance);
      rejectionReason := if accepted == 1.0 then 0 else 3;
      if accepted == 1.0 then
        nextCatalogRevision := catalogRevision;
        prunedCount := prunedCount+evictedCount;
      end if;
    end if;
  end if;
  if accepted <> 1.0 then
    // A later map/anchor failure must undo catalog eviction and reprojection,
    // not merely hold the already prepared geometry.
    point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
    lastSeen := previousLastSeen; lastFrame := previousLastFrame;
    localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
    nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
    nextGeneration := previousGeneration; nextCatalogRevision := previousCatalogRevision;
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
end UpdateCatalogLandmarkMap;
