within SLAM.Simulation;

function observeLandmarks "Synthetic calibrated RGB-D measurements of a fixed scene"
  input Real positionWorld_m[3];
  input Real quaternionWorldBody[4];
  input Real landmarksWorld_m[:, 3];
  input Real rotationBodyCamera[3, 3];
  input Real cameraPositionBody_m[3];
  input Real intrinsics[4] "fx, fy, cx, cy";
  input Integer imageSize_pixels[2] "width, height";
  input Real depthRange_m[2] "Minimum and maximum optical-axis depth";
  input Real measurementError[size(landmarksWorld_m, 1), 3]
    "Additive pixel u, pixel v and optical-depth errors";
  input Boolean detected[size(landmarksWorld_m, 1)];
  input Boolean available = true;
  output Real observations[size(landmarksWorld_m, 1), 3];
  output Boolean visible[size(landmarksWorld_m, 1)];
protected
  Real rotationWorldBody[3, 3];
  Real rotationCameraWorld[3, 3];
  Real cameraPositionWorld_m[3];
  Real pointCamera_m[3];
  Real projected[3];
  Real measured[3];
  Boolean calibrationValid;
  Boolean pointValid;
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(quaternionWorldBody);
  rotationCameraWorld := transpose(rotationWorldBody * rotationBodyCamera);
  cameraPositionWorld_m := positionWorld_m + rotationWorldBody * cameraPositionBody_m;
  calibrationValid := available and imageSize_pixels[1] > 0 and imageSize_pixels[2] > 0
    and intrinsics[1] > 0.0 and intrinsics[2] > 0.0
    and depthRange_m[1] > 0.0 and depthRange_m[2] > depthRange_m[1]
    and depthRange_m[2] < 1.0e30
    and abs(quaternionWorldBody * quaternionWorldBody - 1.0) < 1.0e-3;
  for component in 1:4 loop
    calibrationValid := calibrationValid and abs(intrinsics[component]) < 1.0e30;
  end for;
  observations := zeros(size(landmarksWorld_m, 1), 3);
  visible := fill(false, size(landmarksWorld_m, 1));
  pointCamera_m := zeros(3);
  projected := zeros(3);
  measured := zeros(3);
  pointValid := false;
  for landmark in 1:size(landmarksWorld_m, 1) loop
    pointCamera_m := rotationCameraWorld * (landmarksWorld_m[landmark, :] - cameraPositionWorld_m);
    pointValid := calibrationValid and detected[landmark]
      and pointCamera_m[3] >= depthRange_m[1] and pointCamera_m[3] <= depthRange_m[2];
    for component in 1:3 loop
      pointValid := pointValid and abs(pointCamera_m[component]) < 1.0e30
        and abs(measurementError[landmark, component]) < 1.0e30;
    end for;
    if pointValid then
      projected := {intrinsics[1] * pointCamera_m[1] / pointCamera_m[3] + intrinsics[3],
        intrinsics[2] * pointCamera_m[2] / pointCamera_m[3] + intrinsics[4], pointCamera_m[3]};
      measured := projected + measurementError[landmark, :];
      visible[landmark] := projected[1] >= 0.0 and projected[1] < imageSize_pixels[1]
        and projected[2] >= 0.0 and projected[2] < imageSize_pixels[2]
        and measured[1] >= 0.0 and measured[1] < imageSize_pixels[1]
        and measured[2] >= 0.0 and measured[2] < imageSize_pixels[2]
        and measured[3] >= depthRange_m[1] and measured[3] <= depthRange_m[2];
      if visible[landmark] then
        observations[landmark, :] := measured;
      end if;
    end if;
  end for;
end observeLandmarks;
