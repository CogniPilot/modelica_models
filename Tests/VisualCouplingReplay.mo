within Tests;

block VisualCouplingReplay "Matched Modelica fixed-map loose and tight visual updates"
  input Real quaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  input Real positionWorld_m[3] = zeros(3);
  input Real priorCovariance[15, 15] = 0.001 * identity(15);
  input Real landmarksWorld_m[4, 3] = [-1.0, -1.0, 4.0; 1.0, -1.0, 5.0;
    -1.0, 1.0, 6.0; 1.0, 1.0, 7.0];
  input Real observations[4, 3] = [220.0, 140.0, 4.0; 400.0, 160.0, 5.0;
    253.333333, 306.666667, 6.0; 377.142857, 297.142857, 7.0];
  input Real measurementCovariance[12, 12] = 0.01 * identity(12);
  input Real rotationBodyCamera[3, 3] = 1.0 * identity(3);
  input Real cameraPositionBody_m[3] = zeros(3);
  input Real intrinsics[4] = {400.0, 400.0, 320.0, 240.0};
  output Real looseState[16];
  output Real tightState[16];
  output Real looseCovariance[15, 15];
  output Real tightCovariance[15, 15];
  output Boolean looseAccepted;
  output Boolean tightAccepted;
  output Real looseNis;
  output Real tightNis;
protected
  Estimation.StrapdownINS.ESKF.State predicted(
    positionWorldEnu_m=positionWorld_m, velocityWorldEnu_m_s=zeros(3),
    quaternionWorldBody=quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=zeros(3), accelerometerBiasBodyFlu_m_s2=zeros(3),
    covariance=priorCovariance, useSquareRootCovariance=false, covarianceRoot=zeros(15, 15),
    barometerBiasCrossCovariance=zeros(15), barometerBias_m=0.0,
    barometerBiasVariance_m2=0.0, useJointBarometerBias=false);
  Estimation.StrapdownINS.ESKF.State loose;
  Estimation.StrapdownINS.ESKF.State tight;
algorithm
  when sample(0.0, 0.01) then
    (loose, looseAccepted, looseNis) := SLAM.Fusion.correctMappedLandmarksLoose(
      predicted, landmarksWorld_m, observations, measurementCovariance,
      rotationBodyCamera, cameraPositionBody_m, intrinsics);
    (tight, tightAccepted, tightNis) := SLAM.Fusion.correctMappedLandmarksTight(
      predicted, landmarksWorld_m, observations, measurementCovariance,
      rotationBodyCamera, cameraPositionBody_m, intrinsics);
    looseState := cat(1, loose.positionWorldEnu_m, loose.velocityWorldEnu_m_s,
      loose.quaternionWorldBody, loose.gyroscopeBiasBodyFlu_rad_s, loose.accelerometerBiasBodyFlu_m_s2);
    tightState := cat(1, tight.positionWorldEnu_m, tight.velocityWorldEnu_m_s,
      tight.quaternionWorldBody, tight.gyroscopeBiasBodyFlu_rad_s, tight.accelerometerBiasBodyFlu_m_s2);
    looseCovariance := loose.covariance;
    tightCovariance := tight.covariance;
  end when;
end VisualCouplingReplay;
