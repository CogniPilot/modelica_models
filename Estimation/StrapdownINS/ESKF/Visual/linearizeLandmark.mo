within Estimation.StrapdownINS.ESKF.Visual;

function linearizeLandmark "Calibrated pinhole RGB-D observation in the right ESKF tangent"
  input NominalState nominal;
  input Real landmarkWorld_m[3];
  input Real rotationBodyCamera[3, 3];
  input Real cameraPositionBody_m[3];
  input Real intrinsics[4] "fx, fy, cx, cy";
  input Real minimumDepth_m = 0.1;
  output Real expected[3] "Pixel u, pixel v, optical-axis depth in metres";
  output Real navigationJacobian[3, 15];
  output Real landmarkJacobian[3, 3];
  output Boolean valid;
protected
  Real rotationWorldBody[3, 3];
  Real landmarkBody_m[3];
  Real landmarkCamera_m[3];
  Real projectionJacobian[3, 3];
  Real cameraNavigationJacobian[3, 15];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(nominal.quaternionWorldBody);
  landmarkBody_m := transpose(rotationWorldBody)
    * (landmarkWorld_m - nominal.positionWorldEnu_m);
  landmarkCamera_m := transpose(rotationBodyCamera)
    * (landmarkBody_m - cameraPositionBody_m);
  valid := minimumDepth_m > 0.0 and minimumDepth_m < FiniteMagnitudeLimit
    and intrinsics[1] > 0.0 and intrinsics[2] > 0.0
    and landmarkCamera_m[3] >= minimumDepth_m;
  for component in 1:3 loop
    valid := valid and abs(landmarkCamera_m[component]) < FiniteMagnitudeLimit;
  end for;
  for component in 1:4 loop
    valid := valid and abs(intrinsics[component]) < FiniteMagnitudeLimit;
  end for;
  expected := zeros(3);
  navigationJacobian := zeros(3, 15);
  landmarkJacobian := zeros(3, 3);
  if valid then
    expected := {intrinsics[1] * landmarkCamera_m[1] / landmarkCamera_m[3]
        + intrinsics[3],
      intrinsics[2] * landmarkCamera_m[2] / landmarkCamera_m[3] + intrinsics[4],
      landmarkCamera_m[3]};
    projectionJacobian := [intrinsics[1] / landmarkCamera_m[3], 0.0,
        -intrinsics[1] * landmarkCamera_m[1] / landmarkCamera_m[3]^2;
      0.0, intrinsics[2] / landmarkCamera_m[3],
        -intrinsics[2] * landmarkCamera_m[2] / landmarkCamera_m[3]^2;
      0.0, 0.0, 1.0];
    cameraNavigationJacobian := zeros(3, 15);
    cameraNavigationJacobian[:, 1:3] := -transpose(rotationBodyCamera);
    cameraNavigationJacobian[:, 7:9] := transpose(rotationBodyCamera)
      * LieGroups.SO3.Quat.wedge(landmarkBody_m);
    navigationJacobian := projectionJacobian * cameraNavigationJacobian;
    landmarkJacobian := projectionJacobian * transpose(rotationBodyCamera)
      * transpose(rotationWorldBody);
  end if;
end linearizeLandmark;
