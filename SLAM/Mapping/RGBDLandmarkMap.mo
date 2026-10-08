within SLAM.Mapping;
model RGBDLandmarkMap
  import UpdateLandmarkMap = SLAM.Mapping.UpdateLandmarkMap;

  constant Integer imageHeight = 90;
  constant Integer imageWidth = 160;
  constant Integer mapCapacity = imageHeight*imageWidth;
  constant Integer featureCapacity = 350;
  constant Integer dimension = 3;
  parameter Real coordinateLimit = 1e6;
  parameter Real voxelWidth = 0.25;
  parameter Real mergeRadius = 0.15;
  parameter Real maximumDistance = 80.0;
  parameter Real tentativeLifetime = 0.5;
  parameter Real confirmedLifetime = 5.0;
  parameter Real confirmationObservations = 3.0;
  parameter Real maximumConfidence = 8.0;
  parameter Real maximumTentative = 700.0;
  input Real previousPoint[mapCapacity,dimension] = zeros(mapCapacity,dimension);
  input Real previousOccupied[mapCapacity] = zeros(mapCapacity);
  input Real previousConfidence[mapCapacity] = zeros(mapCapacity);
  input Real previousLastSeen[mapCapacity] = zeros(mapCapacity);
  input Real previousLastFrame[mapCapacity] = zeros(mapCapacity);
  input Real candidatePoint[featureCapacity,dimension] = zeros(featureCapacity,dimension);
  input Real candidateEnabled[featureCapacity] = zeros(featureCapacity);
  input Real candidateCount = 0.0;
  input Real bodyPosition[dimension] = zeros(dimension);
  input Real poseAccepted = 1.0;
  input Real previousTime = 0.0;
  input Real timeNow = 0.0;
  input Real previousFrame = 0.0;
  input Real frameNow = 1.0;
  input Real previousWorldFrame = 0.0;
  input Real worldFrame = 0.0;
  input Real resetRequested = 0.0;
  output Real point[mapCapacity,dimension];
  output Real occupied[mapCapacity];
  output Real confidence[mapCapacity];
  output Real lastSeen[mapCapacity];
  output Real lastFrame[mapCapacity];
  output Real confirmed[mapCapacity];
  output Real accepted; output Real rejectionReason;
  output Real nextTime; output Real nextFrame; output Real nextWorldFrame;
  output Real occupiedCount; output Real confirmedCount; output Real tentativeCount;
  output Real insertedCount; output Real mergedCount; output Real prunedCount;
  output Real droppedCount; output Real invalidCandidateCount;
equation
  (point,occupied,confidence,lastSeen,lastFrame,confirmed,accepted,rejectionReason,nextTime,nextFrame,nextWorldFrame,
    occupiedCount,confirmedCount,tentativeCount,insertedCount,mergedCount,prunedCount,droppedCount,invalidCandidateCount) =
    UpdateLandmarkMap(previousPoint,previousOccupied,previousConfidence,previousLastSeen,previousLastFrame,candidatePoint,candidateEnabled,
      candidateCount,bodyPosition,poseAccepted,previousTime,timeNow,previousFrame,frameNow,previousWorldFrame,worldFrame,resetRequested,
      coordinateLimit,voxelWidth,mergeRadius,maximumDistance,tentativeLifetime,confirmedLifetime,confirmationObservations,maximumConfidence,maximumTentative);
end RGBDLandmarkMap;
