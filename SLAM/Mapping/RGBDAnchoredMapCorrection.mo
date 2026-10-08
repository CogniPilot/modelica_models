within SLAM.Mapping;
model RGBDAnchoredMapCorrection
  import RGBDMapAnchors = SLAM.Mapping.RGBDMapAnchors;

  constant Integer mapCapacity = RGBDMapAnchors.mapCapacity;
  constant Integer keyframeCapacity = RGBDMapAnchors.keyframeCapacity;
  constant Integer dimension = RGBDMapAnchors.dimension;
  parameter Real coordinateLimit = 1e6;
  input Real previousPoint[mapCapacity,dimension] = zeros(mapCapacity,dimension);
  input Real previousLocalPoint[mapCapacity,dimension] = zeros(mapCapacity,dimension);
  input Real previousOccupied[mapCapacity] = zeros(mapCapacity);
  input Integer previousAnchorId[mapCapacity] = fill(0,mapCapacity);
  input Integer previousAnchorSlot[mapCapacity] = fill(0,mapCapacity);
  input Integer mapGeneration = 1; input Integer previousRevision = 0;
  input Boolean nodeEnabled[keyframeCapacity] = fill(false,keyframeCapacity);
  input Integer nodeId[keyframeCapacity] = fill(0,keyframeCapacity);
  input Real nodePosition[keyframeCapacity,dimension] = zeros(keyframeCapacity,dimension);
  input Real nodeRotation[keyframeCapacity,dimension,dimension] = zeros(keyframeCapacity,dimension,dimension);
  input Integer graphGeneration = 1; input Integer graphRevision = 1;
  input Boolean graphAccepted = false; input Boolean requested = false;
  output Real point[mapCapacity,dimension]; output Real localPoint[mapCapacity,dimension];
  output Real occupied[mapCapacity];
  output Integer anchorId[mapCapacity]; output Integer anchorSlot[mapCapacity];
  output Integer nextRevision; output Boolean accepted;
  output Integer rejectionReason; output Integer projectedCount; output Integer prunedCount;
equation
  (point,localPoint,occupied,anchorId,anchorSlot,nextRevision,accepted,rejectionReason,projectedCount,prunedCount) =
    RGBDMapAnchors.Reproject(previousPoint,previousLocalPoint,previousOccupied,previousAnchorId,previousAnchorSlot,
      mapGeneration,previousRevision,nodeEnabled,nodeId,nodePosition,nodeRotation,graphGeneration,graphRevision,
      graphAccepted,requested,coordinateLimit);
end RGBDAnchoredMapCorrection;
