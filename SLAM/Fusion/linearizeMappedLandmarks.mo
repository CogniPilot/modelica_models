within SLAM.Fusion;

function linearizeMappedLandmarks "Shared RGB-D residuals for fixed, known map landmarks"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Real landmarksWorld_m[:, 3];
  input Real observations[size(landmarksWorld_m, 1), 3];
  input Real rotationBodyCamera[3, 3];
  input Real cameraPositionBody_m[3];
  input Real intrinsics[4];
  output Real residual[3 * size(landmarksWorld_m, 1)];
  output Real navigationJacobian[3 * size(landmarksWorld_m, 1), 15];
  output Boolean valid;
protected
  Estimation.StrapdownINS.ESKF.NominalState nominal;
  Real expected[3];
  Real featureJacobian[3, 15];
  Real landmarkJacobian[3, 3];
  Boolean featureValid;
algorithm
  nominal := Estimation.StrapdownINS.ESKF.NominalState(
    positionWorldEnu_m=predicted.positionWorldEnu_m,
    velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
    quaternionWorldBody=predicted.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2);
  residual := zeros(3 * size(landmarksWorld_m, 1));
  navigationJacobian := zeros(3 * size(landmarksWorld_m, 1), 15);
  expected := zeros(3);
  featureJacobian := zeros(3, 15);
  landmarkJacobian := zeros(3, 3);
  featureValid := false;
  valid := size(landmarksWorld_m, 1) > 0;
  for feature in 1:size(landmarksWorld_m, 1) loop
    (expected, featureJacobian, landmarkJacobian, featureValid) :=
      Estimation.StrapdownINS.ESKF.Visual.linearizeLandmark(nominal,
        landmarksWorld_m[feature, :], rotationBodyCamera, cameraPositionBody_m, intrinsics);
    for component in 1:3 loop
      residual[3 * (feature - 1) + component] := observations[feature, component] - expected[component];
      for tangent in 1:15 loop
        navigationJacobian[3 * (feature - 1) + component, tangent] := featureJacobian[component, tangent];
      end for;
    end for;
    valid := valid and featureValid;
  end for;
end linearizeMappedLandmarks;
