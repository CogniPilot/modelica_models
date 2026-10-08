within SLAM.Localization;
// Convert accepted camera-frame registration to a map-frame BODY observation.
// The reference pose is the retained estimator/keyframe pose, never truth.
// Registration convention: currentPoint = currentFromReference * referencePoint
// + currentFromReferenceTranslation. Both point clouds use optical RDF axes.
// Camera extrinsics map optical coordinates into body FLU coordinates.
// This component composes a measurement; it does not estimate covariance,
// persist keyframes, reject dynamic objects, or implement loop closure.
model RGBDRelativePose
  import RGBDProperRotation = SLAM.Localization.RGBDProperRotation;

  constant Integer dimension = 3;
  input Real referenceBodyRotation[dimension,dimension] = identity(dimension);
  input Real referenceBodyPosition[dimension] = zeros(dimension);
  input Real currentFromReference[dimension,dimension] = identity(dimension);
  input Real currentFromReferenceTranslation[dimension] = zeros(dimension);
  input Real opticalToBody[dimension,dimension] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[dimension] = {0.18,0.0,-0.04};
  input Real registrationAccepted = 0.0;
  output Real valid;
  output Real observedBodyRotation[dimension,dimension];
  output Real observedBodyPosition[dimension];
protected
  parameter Real coordinateLimit = 1e6;
  RGBDProperRotation referenceCheck(rotation=referenceBodyRotation);
  RGBDProperRotation registrationCheck(rotation=currentFromReference);
  RGBDProperRotation extrinsicsCheck(rotation=opticalToBody);
  Real positionChecks[dimension];
  Real referenceCameraRotation[dimension,dimension];
  Real referenceCameraPosition[dimension];
  Real currentCameraRotation[dimension,dimension];
  Real currentCameraPosition[dimension];
  Real proposedBodyRotation[dimension,dimension];
  Real proposedBodyPosition[dimension];
equation
  referenceCameraRotation = referenceBodyRotation*opticalToBody;
  referenceCameraPosition = referenceBodyPosition+referenceBodyRotation*cameraOriginBody;
  currentCameraRotation = referenceCameraRotation*transpose(currentFromReference);
  currentCameraPosition = referenceCameraPosition-currentCameraRotation*currentFromReferenceTranslation;
  proposedBodyRotation = currentCameraRotation*transpose(opticalToBody);
  proposedBodyPosition = currentCameraPosition-proposedBodyRotation*cameraOriginBody;
  for i in 1:dimension loop
    positionChecks[i] = if noEvent(abs(referenceBodyPosition[i]) <= coordinateLimit
      and abs(currentFromReferenceTranslation[i]) <= coordinateLimit
      and abs(cameraOriginBody[i]) <= coordinateLimit
      and abs(proposedBodyPosition[i]) <= coordinateLimit) then 0.0 else 1.0;
  end for;
  valid = if noEvent(registrationAccepted >= 1.0 and registrationAccepted <= 1.0
    and referenceCheck.valid > 0.5 and registrationCheck.valid > 0.5
    and extrinsicsCheck.valid > 0.5 and sum(positionChecks) < 0.5) then 1.0 else 0.0;
  observedBodyRotation = if noEvent(valid > 0.5) then proposedBodyRotation else identity(dimension);
  observedBodyPosition = if noEvent(valid > 0.5) then proposedBodyPosition else zeros(dimension);
end RGBDRelativePose;
