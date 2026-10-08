within Tests;

block SyntheticLandmarkReplay "Synthetic landmark camera qualification entrypoint"
  input Real positionWorld_m[3];
  input Real quaternionWorldBody[4];
  input Real landmarksWorld_m[4, 3];
  input Real rotationBodyCamera[3, 3];
  input Real cameraPositionBody_m[3];
  input Real intrinsics[4];
  input Integer imageSize_pixels[2];
  input Real depthRange_m[2];
  input Real measurementError[4, 3];
  input Boolean detected[4];
  input Boolean available;
  output Real observations[4, 3];
  output Boolean visible[4];
  output Real grid[4, 3];
  output Real defaultObservations[12, 3];
  output Boolean defaultVisible[12];
  output Integer defaultLandmarkIds[12];
protected
  SLAM.Simulation.LandmarkCamera defaultCamera(
    positionWorld_m={0.0, 0.0, 4.0}, quaternionWorldBody={1.0, 0.0, 0.0, 0.0});
  SLAM.Simulation.LandmarkCamera camera(landmarkCount=4,
    landmarksWorld_m=landmarksWorld_m, positionWorld_m=positionWorld_m,
    quaternionWorldBody=quaternionWorldBody, rotationBodyCamera=rotationBodyCamera,
    cameraPositionBody_m=cameraPositionBody_m, intrinsics=intrinsics,
    imageSize_pixels=imageSize_pixels, depthRange_m=depthRange_m,
    measurementError=measurementError, detected=detected, available=available);
equation
  observations = camera.observations;
  visible = camera.visible;
  defaultObservations = defaultCamera.observations;
  defaultVisible = defaultCamera.visible;
  defaultLandmarkIds = defaultCamera.landmarkIds;
algorithm
  when sample(0.0, 0.1) then
    grid := SLAM.Simulation.landmarkGrid(2, 2, 2.0);
  end when;
end SyntheticLandmarkReplay;
