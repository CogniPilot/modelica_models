within Tests;

block VisualNavigationReplay "Sequential fixed-map visual and GPS aiding comparison"
  input Boolean tightCoupling = false;
  input Real previousNominal[16];
  input Real previousCovariance[15, 15];
  input Real angularVelocityBody_rad_s[3];
  input Real specificForceBody_m_s2[3];
  input Real interval_s;
  input Real imuNoiseDensity[2];
  input Boolean gpsAvailable;
  input Boolean cameraAvailable;
  input Real gpsPositionWorld_m[3];
  input Real gpsVelocityWorld_m_s[3];
  input Real gpsStandardDeviation[2];
  input Real landmarksWorld_m[4, 3];
  input Real observations[4, 3];
  input Real measurementCovariance[12, 12];
  input Real rotationBodyCamera[3, 3];
  input Real cameraPositionBody_m[3];
  input Real intrinsics[4];
  output Real nominal[16];
  output Real covariance[15, 15];
  output Boolean gpsAccepted;
  output Boolean cameraAccepted;
  output Real gpsNis;
  output Real cameraNis;
protected
  Estimation.StrapdownINS.ESKF.State previous(
    positionWorldEnu_m=previousNominal[1:3],
    velocityWorldEnu_m_s=previousNominal[4:6],
    quaternionWorldBody=previousNominal[7:10],
    gyroscopeBiasBodyFlu_rad_s=previousNominal[11:13],
    accelerometerBiasBodyFlu_m_s2=previousNominal[14:16],
    covariance=previousCovariance, useSquareRootCovariance=false,
    covarianceRoot=zeros(15, 15), barometerBiasCrossCovariance=zeros(15),
    barometerBias_m=0.0, barometerBiasVariance_m2=0.0, useJointBarometerBias=false);
  Estimation.StrapdownINS.ProcessNoise processNoise(
    gyroscope_rad2_s=imuNoiseDensity[1]^2 * identity(3),
    accelerometer_m2_s3=imuNoiseDensity[2]^2 * identity(3),
    gyroscopeBias_rad2_s3=zeros(3, 3), accelerometerBias_m2_s5=zeros(3, 3));
  Avionics.GpsSample gps(
    valid=gpsAvailable, fresh=gpsAvailable, positionValid=gpsAvailable,
    velocityValid=gpsAvailable, timestamp_s=0.0, geodetic_deg_m=zeros(3),
    positionWorldEnu_m=gpsPositionWorld_m,
    velocityWorldEnu_m_s=gpsVelocityWorld_m_s,
    positionCovarianceWorld_m2=gpsStandardDeviation[1]^2 * identity(3),
    velocityCovarianceWorld_m2_s2=gpsStandardDeviation[2]^2 * identity(3));
  Estimation.StrapdownINS.ESKF.State predicted;
  Estimation.StrapdownINS.ESKF.State gpsCorrected;
  Estimation.StrapdownINS.ESKF.State cameraCorrected;
  Integer rejectionReason;
algorithm
  when sample(0.0, 0.01) then
    predicted := Estimation.StrapdownINS.ESKF.predict(previous,
      angularVelocityBody_rad_s, specificForceBody_m_s2,
      {0.0, 0.0, -9.81}, interval_s, processNoise);
    if gpsAvailable then
      (gpsCorrected, gpsAccepted, rejectionReason, gpsNis) :=
        Estimation.StrapdownINS.ESKF.correctGps(predicted, gps);
    else
      gpsCorrected := Estimation.StrapdownINS.ESKF.copyState(predicted);
      gpsAccepted := false;
      gpsNis := 0.0;
      rejectionReason := 0;
    end if;
    if cameraAvailable then
      if tightCoupling then
        (cameraCorrected, cameraAccepted, cameraNis) := SLAM.Fusion.correctMappedLandmarksTight(
          gpsCorrected, landmarksWorld_m, observations, measurementCovariance,
          rotationBodyCamera, cameraPositionBody_m, intrinsics);
      else
        (cameraCorrected, cameraAccepted, cameraNis) := SLAM.Fusion.correctMappedLandmarksLoose(
          gpsCorrected, landmarksWorld_m, observations, measurementCovariance,
          rotationBodyCamera, cameraPositionBody_m, intrinsics);
      end if;
    else
      cameraCorrected := Estimation.StrapdownINS.ESKF.copyState(gpsCorrected);
      cameraAccepted := false;
      cameraNis := 0.0;
    end if;
    nominal := cat(1, cameraCorrected.positionWorldEnu_m, cameraCorrected.velocityWorldEnu_m_s,
      cameraCorrected.quaternionWorldBody, cameraCorrected.gyroscopeBiasBodyFlu_rad_s,
      cameraCorrected.accelerometerBiasBodyFlu_m_s2);
    covariance := cameraCorrected.covariance;
  end when;
end VisualNavigationReplay;
