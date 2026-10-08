within SLAM.Mapping;
// Candidate coordinates only: no map persistence, pruning or SLAM lifecycle.
// Points are already calibrated optical RDF (right/down/forward). The supplied
// estimated body pose maps body FLU into world ENU; there is no truth input.
model RGBDLandmarkProjection
  import RGBDLandmarkRotationCheck = SLAM.Mapping.RGBDLandmarkRotationCheck;

  constant Integer featureCapacity = 350;
  constant Integer dimension = 3;
  parameter Real coordinateLimit = 1e6;
  input Real opticalPoint[featureCapacity,dimension];
  input Real enabled[featureCapacity];
  input Real activeCount = 0.0;
  input Real poseAccepted = 0.0;
  input Real bodyRotation[dimension,dimension] = identity(dimension);
  input Real bodyPosition[dimension] = zeros(dimension);
  input Real opticalToBody[dimension,dimension] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[dimension] = {0.18,0.0,-0.04};
  output Real worldPoint[featureCapacity,dimension];
  output Real landmarkEnabled[featureCapacity];
  output Real validCount;
  output Real invalidCount;
  output Real configurationValid;
  output Real poseValid;
protected
  RGBDLandmarkRotationCheck bodyCheck(rotation=bodyRotation);
  RGBDLandmarkRotationCheck cameraCheck(rotation=opticalToBody);
  Real poseChecks[dimension];
  Real pointChecks[featureCapacity,dimension];
  Real pointValid[featureCapacity];
  Real invalidChecks[featureCapacity];
  Real safeOptical[featureCapacity,dimension];
  Real safeBodyRotation[dimension,dimension];
  Real safeOpticalToBody[dimension,dimension];
  Real safeBodyPosition[dimension];
  Real safeCameraOrigin[dimension];
  Real cameraInBody[featureCapacity,dimension];
  Real proposedWorld[featureCapacity,dimension];
  Real outputChecks[featureCapacity,dimension];
equation
  configurationValid = if noEvent(activeCount >= 0.0 and activeCount <= featureCapacity
    and floor(activeCount) <= activeCount and floor(activeCount) >= activeCount
    and coordinateLimit > 0.0 and coordinateLimit <= 1e6) then 1.0 else 0.0;
  for k in 1:dimension loop
    poseChecks[k] = if noEvent(abs(bodyPosition[k]) <= coordinateLimit
      and abs(cameraOriginBody[k]) <= coordinateLimit) then 0.0 else 1.0;
  end for;
  poseValid = if noEvent(configurationValid > 0.5 and poseAccepted >= 1.0 and poseAccepted <= 1.0
    and bodyCheck.valid > 0.5 and cameraCheck.valid > 0.5 and sum(poseChecks) < 0.5) then 1.0 else 0.0;
  safeBodyRotation = if noEvent(poseValid > 0.5) then bodyRotation else identity(dimension);
  safeOpticalToBody = if noEvent(poseValid > 0.5) then opticalToBody else identity(dimension);
  safeBodyPosition = if noEvent(poseValid > 0.5) then bodyPosition else zeros(dimension);
  safeCameraOrigin = if noEvent(poseValid > 0.5) then cameraOriginBody else zeros(dimension);
  for i in 1:featureCapacity loop
    for k in 1:dimension loop
      pointChecks[i,k] = if noEvent(abs(opticalPoint[i,k]) <= coordinateLimit) then 0.0 else 1.0;
      safeOptical[i,k] = if noEvent(poseValid > 0.5 and pointValid[i] > 0.5) then opticalPoint[i,k] else 0.0;
      outputChecks[i,k] = if noEvent(abs(proposedWorld[i,k]) <= coordinateLimit) then 0.0 else 1.0;
      worldPoint[i,k] = if noEvent(landmarkEnabled[i] > 0.5) then proposedWorld[i,k] else 0.0;
    end for;
    pointValid[i] = if noEvent(configurationValid > 0.5 and i <= activeCount
      and enabled[i] >= 1.0 and enabled[i] <= 1.0 and sum(pointChecks[i,:]) < 0.5
      and opticalPoint[i,3] > 0.0) then 1.0 else 0.0;
    cameraInBody[i,:] = safeOpticalToBody*safeOptical[i,:]+safeCameraOrigin;
    proposedWorld[i,:] = safeBodyRotation*cameraInBody[i,:]+safeBodyPosition;
    landmarkEnabled[i] = if noEvent(poseValid > 0.5 and pointValid[i] > 0.5
      and sum(outputChecks[i,:]) < 0.5) then 1.0 else 0.0;
    invalidChecks[i] = if noEvent(i <= activeCount and not (enabled[i] >= 0.0 and enabled[i] <= 0.0)
      and pointValid[i] < 0.5) then 1.0 else 0.0;
  end for;
  validCount = sum(landmarkEnabled);
  invalidCount = sum(invalidChecks);
end RGBDLandmarkProjection;
