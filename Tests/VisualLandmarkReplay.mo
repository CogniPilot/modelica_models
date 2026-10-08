within Tests;

block VisualLandmarkReplay "RGB-D observation and Lie injection qualification"
  input Real quaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  input Real positionWorld_m[3] = zeros(3);
  input Real landmarkWorld_m[3] = {0.0, 0.0, 5.0};
  input Real rotationBodyCamera[3, 3] = 1.0 * identity(3);
  input Real cameraPositionBody_m[3] = zeros(3);
  input Real intrinsics[4] = {400.0, 400.0, 320.0, 240.0};
  input Real correction[15] = zeros(15);
  output Real expected[3];
  output Real navigationJacobian[3, 15];
  output Real landmarkJacobian[3, 3];
  output Boolean valid;
protected
  Estimation.StrapdownINS.ESKF.NominalState nominal(
    positionWorldEnu_m=positionWorld_m, velocityWorldEnu_m_s=zeros(3),
    quaternionWorldBody=quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=zeros(3), accelerometerBiasBodyFlu_m_s2=zeros(3));
  Estimation.StrapdownINS.ESKF.NominalState corrected;
algorithm
  when sample(0.0, 0.01) then
    corrected := Estimation.StrapdownINS.ESKF.inject(nominal, correction);
    (expected, navigationJacobian, landmarkJacobian, valid) :=
      Estimation.StrapdownINS.ESKF.Visual.linearizeLandmark(corrected,
        landmarkWorld_m, rotationBodyCamera, cameraPositionBody_m, intrinsics);
  end when;
end VisualLandmarkReplay;
