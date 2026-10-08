within SLAM.Mapping;
// Existing public numerical interface delegates to the same kernel. Receipts
// are consumed by anchored composition; old callers retain their exact outputs.
function UpdateLandmarkMap
  import RGBDLandmarkMapInterface = SLAM.Mapping.RGBDLandmarkMapInterface;
  import UpdateLandmarkMapWithReceipts = SLAM.Mapping.UpdateLandmarkMapWithReceipts;

  extends RGBDLandmarkMapInterface;
protected
  Integer unusedReceipt[size(previousOccupied,1)];
algorithm
  (point,occupied,confidence,lastSeen,lastFrame,confirmed,accepted,rejectionReason,nextTime,nextFrame,nextWorldFrame,
    occupiedCount,confirmedCount,tentativeCount,insertedCount,mergedCount,prunedCount,droppedCount,invalidCandidateCount,
    unusedReceipt) := UpdateLandmarkMapWithReceipts(
      previousPoint,previousOccupied,previousConfidence,previousLastSeen,previousLastFrame,candidatePoint,candidateEnabled,
      candidateCount,bodyPosition,poseAccepted,previousTime,timeNow,previousFrame,frameNow,previousWorldFrame,worldFrame,resetRequested,
      coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,confirmationObservations,
      maximumConfidence,maximumTentative);
end UpdateLandmarkMap;
