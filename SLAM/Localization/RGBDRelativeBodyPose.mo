within SLAM.Localization;
// Ordered callable equivalent of the unchanged equation model above.
// This preserves its optical reference->current transform and covariance chart.
function RGBDRelativeBodyPose
  import RGBDProperRotationValue = SLAM.Localization.RGBDProperRotationValue;

  input Real referenceBodyRotation[3,3] = identity(3);
  input Real referenceBodyPosition[3] = zeros(3);
  input Real currentFromReference[3,3] = identity(3);
  input Real currentFromReferenceTranslation[3] = zeros(3);
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real registrationAccepted = 0.0;
  output Real valid;
  output Real observedBodyRotation[3,3];
  output Real observedBodyPosition[3];
protected
  constant Real coordinateLimit = 1e6;
  Real referenceCameraRotation[3,3]; Real referenceCameraPosition[3];
  Real currentCameraRotation[3,3]; Real currentCameraPosition[3];
  Real proposedBodyRotation[3,3]; Real proposedBodyPosition[3];
  Real positionChecks;
algorithm
  referenceCameraRotation := referenceBodyRotation*opticalToBody;
  referenceCameraPosition := referenceBodyPosition+referenceBodyRotation*cameraOriginBody;
  currentCameraRotation := referenceCameraRotation*transpose(currentFromReference);
  currentCameraPosition := referenceCameraPosition-currentCameraRotation*currentFromReferenceTranslation;
  proposedBodyRotation := currentCameraRotation*transpose(opticalToBody);
  proposedBodyPosition := currentCameraPosition-proposedBodyRotation*cameraOriginBody;
  positionChecks := 0.0;
  for i in 1:3 loop
    if not (abs(referenceBodyPosition[i]) <= coordinateLimit
      and abs(currentFromReferenceTranslation[i]) <= coordinateLimit
      and abs(cameraOriginBody[i]) <= coordinateLimit
      and abs(proposedBodyPosition[i]) <= coordinateLimit) then
      positionChecks := positionChecks+1.0;
    end if;
  end for;
  valid := if registrationAccepted >= 1.0 and registrationAccepted <= 1.0
    and RGBDProperRotationValue(referenceBodyRotation) > 0.5
    and RGBDProperRotationValue(currentFromReference) > 0.5
    and RGBDProperRotationValue(opticalToBody) > 0.5 and positionChecks < 0.5 then 1.0 else 0.0;
  observedBodyRotation := if valid > 0.5 then proposedBodyRotation else identity(3);
  observedBodyPosition := if valid > 0.5 then proposedBodyPosition else zeros(3);
end RGBDRelativeBodyPose;
