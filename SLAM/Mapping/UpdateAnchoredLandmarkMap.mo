within SLAM.Mapping;
// Geometry, observation metadata and keyframe-local anchors share one commit.
// This owner consumes a coherent catalog proposal; catalog/filter/graph commits
// must still be coordinated by the enclosing SLAM transaction.
function UpdateAnchoredLandmarkMap
  import AssignLandmarkAnchors = SLAM.Mapping.AssignLandmarkAnchors;
  import RGBDLandmarkMapInterface = SLAM.Mapping.RGBDLandmarkMapInterface;
  import UpdateLandmarkMapWithReceipts = SLAM.Mapping.UpdateLandmarkMapWithReceipts;

  extends RGBDLandmarkMapInterface;
  input Real previousLocalPoint[size(previousOccupied,1),3];
  input Integer previousAnchorId[size(previousOccupied,1)];
  input Integer previousAnchorSlot[size(previousOccupied,1)];
  input Integer previousGeneration;
  input Boolean nodeEnabled[:]; input Integer nodeId[size(nodeEnabled,1)];
  input Real nodePosition[size(nodeEnabled,1),3]; input Real nodeRotation[size(nodeEnabled,1),3,3];
  input Integer catalogGeneration; input Integer generation;
  input Integer selectedAnchorId; input Integer selectedAnchorSlot;
  input Boolean requested; input Real consistencyTolerance;
  output Real localPoint[size(previousOccupied,1),3];
  output Integer anchorId[size(previousOccupied,1)]; output Integer anchorSlot[size(previousOccupied,1)];
  output Integer nextGeneration;
  output Real mapRejectionReason; output Integer anchorRejectionReason;
  output Integer assignedCount; output Integer retainedCount; output Integer clearedCount;
protected
  Integer insertedFeature[size(previousOccupied,1)];
  Boolean anchorsAccepted;
algorithm
  point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
  lastSeen := previousLastSeen; lastFrame := previousLastFrame;
  nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
  localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
  nextGeneration := previousGeneration;
  accepted := 0.0; rejectionReason := 1.0; mapRejectionReason := 0.0; anchorRejectionReason := 0;
  assignedCount := 0; retainedCount := 0; clearedCount := 0;
  insertedCount := 0.0; mergedCount := 0.0; prunedCount := 0.0;
  droppedCount := 0.0; invalidCandidateCount := 0.0;
  if requested then
    (point,occupied,confidence,lastSeen,lastFrame,confirmed,accepted,mapRejectionReason,nextTime,nextFrame,nextWorldFrame,
      occupiedCount,confirmedCount,tentativeCount,insertedCount,mergedCount,prunedCount,droppedCount,invalidCandidateCount,
      insertedFeature) := UpdateLandmarkMapWithReceipts(
        previousPoint,previousOccupied,previousConfidence,previousLastSeen,previousLastFrame,candidatePoint,candidateEnabled,
        candidateCount,bodyPosition,poseAccepted,previousTime,timeNow,previousFrame,frameNow,previousWorldFrame,worldFrame,resetRequested,
        coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,confirmationObservations,
        maximumConfidence,maximumTentative);
    rejectionReason := 2.0;
    if accepted == 1.0 then
      (localPoint,anchorId,anchorSlot,nextGeneration,anchorsAccepted,anchorRejectionReason,
        assignedCount,retainedCount,clearedCount) := AssignLandmarkAnchors(
          previousPoint,previousOccupied,previousLocalPoint,previousAnchorId,previousAnchorSlot,previousGeneration,
          point,occupied,insertedFeature,candidatePoint,candidateEnabled,candidateCount,
          nodeEnabled,nodeId,nodePosition,nodeRotation,catalogGeneration,generation,selectedAnchorId,selectedAnchorSlot,
          true,true,resetRequested == 1.0,coordinateLimit,consistencyTolerance);
      accepted := if anchorsAccepted then 1.0 else 0.0;
      rejectionReason := if anchorsAccepted then 0.0 else 3.0;
    end if;
  end if;
  if accepted <> 1.0 then
    // Even a failure discovered at the final landmark rolls back the map's
    // merges, pruning, insertion and clocks, as well as every anchor field.
    point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
    lastSeen := previousLastSeen; lastFrame := previousLastFrame;
    nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
    localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
    nextGeneration := previousGeneration;
    insertedCount := 0.0; mergedCount := 0.0; prunedCount := 0.0;
    droppedCount := 0.0; invalidCandidateCount := 0.0;
    assignedCount := 0; retainedCount := 0; clearedCount := 0;
    // Accepted counts come directly from the map kernel. Recompute only on
    // rollback so diagnostics cannot describe its discarded tentative state.
    occupiedCount := 0.0; confirmedCount := 0.0; tentativeCount := 0.0;
    for slot in 1:size(previousOccupied,1) loop
      confirmed[slot] := if occupied[slot] == 1.0 and confidence[slot] >= confirmationObservations then 1.0 else 0.0;
      occupiedCount := occupiedCount+(if occupied[slot] == 1.0 then 1.0 else 0.0);
      confirmedCount := confirmedCount+confirmed[slot];
      tentativeCount := tentativeCount+(if occupied[slot] == 1.0 and confirmed[slot] == 0.0 then 1.0 else 0.0);
    end for;
  end if;
end UpdateAnchoredLandmarkMap;
