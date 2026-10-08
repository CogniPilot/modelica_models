within SLAM.Mapping;
// Source-owned reusable projection; the unchanged equation model is its oracle.
// Counts and masks retain the model's exact Real-domain checks. Disabled or
// unavailable point payloads never enter transforms. World-output overflow is
// excluded from validCount but is not an optical-input invalidCount event.
function RGBDProjectLandmarks
  import RGBDLandmarkRotationValid = SLAM.Mapping.RGBDLandmarkRotationValid;

  input Real opticalPoint[:,3];
  input Real enabled[size(opticalPoint,1)];
  input Real activeCount = 0.0;
  input Real poseAccepted = 0.0;
  input Real bodyRotation[3,3] = identity(3);
  input Real bodyPosition[3] = zeros(3);
  input Real opticalToBody[3,3] = [0.0,0.0,1.0;-1.0,0.0,0.0;0.0,-1.0,0.0];
  input Real cameraOriginBody[3] = {0.18,0.0,-0.04};
  input Real coordinateLimit = 1e6;
  output Real worldPoint[size(opticalPoint,1),3];
  output Real landmarkEnabled[size(opticalPoint,1)];
  output Real validCount;
  output Real invalidCount;
  output Real configurationValid;
  output Real poseValid;
protected
  Real bodyValid; Real cameraValid; Real poseChecks;
  Real pointChecks; Real pointValid; Real outputChecks;
  Real cameraInBody[3]; Real proposedWorld[3];
algorithm
  worldPoint := zeros(size(opticalPoint,1),3);
  landmarkEnabled := zeros(size(opticalPoint,1)); validCount := 0.0; invalidCount := 0.0;
  configurationValid := if activeCount >= 0.0 and activeCount <= size(opticalPoint,1)
    and floor(activeCount) <= activeCount and floor(activeCount) >= activeCount
    and coordinateLimit > 0.0 and coordinateLimit <= 1e6 then 1.0 else 0.0;
  bodyValid := RGBDLandmarkRotationValid(bodyRotation);
  cameraValid := RGBDLandmarkRotationValid(opticalToBody);
  poseChecks := 0.0;
  for k in 1:3 loop
    if not (abs(bodyPosition[k]) <= coordinateLimit and abs(cameraOriginBody[k]) <= coordinateLimit) then
      poseChecks := poseChecks+1.0;
    end if;
  end for;
  poseValid := if configurationValid > 0.5 and poseAccepted >= 1.0 and poseAccepted <= 1.0
    and bodyValid > 0.5 and cameraValid > 0.5 and poseChecks < 0.5 then 1.0 else 0.0;
  pointChecks := 0.0; pointValid := 0.0; outputChecks := 0.0;
  cameraInBody := zeros(3); proposedWorld := zeros(3);
  for i in 1:size(opticalPoint,1) loop
    pointChecks := 0.0;
    for k in 1:3 loop
      if not (abs(opticalPoint[i,k]) <= coordinateLimit) then pointChecks := pointChecks+1.0; end if;
    end for;
    pointValid := if configurationValid > 0.5 and i <= activeCount
      and enabled[i] >= 1.0 and enabled[i] <= 1.0 and pointChecks < 0.5
      and opticalPoint[i,3] > 0.0 then 1.0 else 0.0;
    if i <= activeCount and not (enabled[i] >= 0.0 and enabled[i] <= 0.0)
      and pointValid < 0.5 then invalidCount := invalidCount+1.0; end if;
    if poseValid > 0.5 and pointValid > 0.5 then
      cameraInBody := opticalToBody*opticalPoint[i,:]+cameraOriginBody;
      proposedWorld := bodyRotation*cameraInBody+bodyPosition;
      outputChecks := 0.0;
      for k in 1:3 loop
        if not (abs(proposedWorld[k]) <= coordinateLimit) then outputChecks := outputChecks+1.0; end if;
      end for;
      if outputChecks < 0.5 then
        worldPoint[i,:] := proposedWorld; landmarkEnabled[i] := 1.0; validCount := validCount+1.0;
      end if;
    end if;
  end for;
end RGBDProjectLandmarks;
