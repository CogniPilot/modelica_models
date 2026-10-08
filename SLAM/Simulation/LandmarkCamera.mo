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
  annotation(Documentation(info = "<html>
    <p>Sampled pinhole RGB-D camera. Supply a fixed landmark scene, true
    world-ENU position and unit body-to-world quaternion. The default optical
    axis points downward for a level FLU body.</p>
    <h4>Geometry</h4>
    <p>Configure horizontal/vertical <code>fieldOfView_rad</code>,
    <code>imageSize_pixels</code> and optical-axis <code>depthRange_m</code>.
    Default focal lengths are width/(2 tan(horizontalFov/2)) and
    height/(2 tan(verticalFov/2)); measured <code>intrinsics</code> may override
    them. <code>rotationBodyCamera</code> maps optical-camera vectors to body
    FLU; <code>cameraPositionBody_m</code> is the lever arm.</p>
    <h4>Observations and visibility</h4>
    <p>Each observation row is {pixel u, pixel v, optical depth in metres}.
    Both ideal and noisy observations must lie inside the image's half-open
    bounds and inclusive depth range. <code>visible</code> is authoritative:
    invisible rows contain zeros and must not be fused. <code>landmarkIds</code>
    are stable row indices; preserve an external feature-ID mapping if rows
    from a captured map are reordered.</p>
    <p>Supply additive <code>measurementError</code> explicitly to share noise
    across experiments. The caller owns its covariance and timestamps.
    <code>detected</code> supplies per-row dropout; <code>available</code>
    disables the camera. The model does not infer occlusion, distortion,
    rolling shutter or association ambiguity.</p>
    <h4>Example</h4>
    <pre>
SLAM.Simulation.LandmarkCamera camera(
  landmarkCount=4,
  landmarksWorld_m=SLAM.Simulation.landmarkGrid(2, 2, 2.0),
  positionWorld_m=truthPositionWorld_m,
  quaternionWorldBody=truthQuaternionWorldBody,
  fieldOfView_rad={1.4, 1.0}, depthRange_m={0.2, 8.0});
    </pre>
    </html>"));
end LandmarkCamera;
