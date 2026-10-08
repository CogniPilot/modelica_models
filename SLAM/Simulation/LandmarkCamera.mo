within SLAM.Simulation;

block LandmarkCamera "Landmark-level RGB-D camera with explicit visibility and errors"
  parameter Integer landmarkCount(min=1) = 12;
  input Real landmarksWorld_m[landmarkCount, 3] = landmarkGrid(3, 4, 0.75)
    "Fixed scene truth; may be supplied from an independently initialized map";
  parameter Real cameraRate_Hz(min=1.0e-6) = 10.0;
  parameter Real fieldOfView_rad[2] = {2.0 * atan(640.0 / 700.0), 2.0 * atan(480.0 / 700.0)};
  input Real intrinsics[4] = {imageSize_pixels[1] / (2.0 * tan(0.5 * fieldOfView_rad[1])),
    imageSize_pixels[2] / (2.0 * tan(0.5 * fieldOfView_rad[2])),
    0.5 * imageSize_pixels[1], 0.5 * imageSize_pixels[2]};
  input Integer imageSize_pixels[2] = {640, 480};
  input Real depthRange_m[2] = {0.1, 10.0};
  input Real rotationBodyCamera[3, 3] = [1.0, 0.0, 0.0; 0.0, -1.0, 0.0; 0.0, 0.0, -1.0];
  input Real cameraPositionBody_m[3] = zeros(3);
  input Real positionWorld_m[3];
  input Real quaternionWorldBody[4];
  input Real measurementError[landmarkCount, 3] = zeros(landmarkCount, 3);
  input Boolean detected[landmarkCount] = fill(true, landmarkCount);
  input Boolean available = true;
  output Integer landmarkIds[landmarkCount] = 1:landmarkCount;
  output Real observations[landmarkCount, 3];
  output Boolean visible[landmarkCount];
algorithm
  when sample(0.0, 1.0 / cameraRate_Hz) then
    (observations, visible) := observeLandmarks(positionWorld_m, quaternionWorldBody,
      landmarksWorld_m, rotationBodyCamera, cameraPositionBody_m, intrinsics,
      imageSize_pixels, depthRange_m, measurementError, detected, available);
  end when;
end LandmarkCamera;
