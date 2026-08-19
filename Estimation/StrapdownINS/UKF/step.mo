within Estimation.StrapdownINS.UKF;

function step
  "Execute one sampled UKF prediction/correction tick"
  input Boolean initializedPrevious;
  input Estimation.StrapdownINS.UKF.State previous;
  input Boolean reset;
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.OpticalFlowSample opticalFlow;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt(unit = "s");
  input Real initialPositionWorldEnu_m[3];
  input Real initialVelocityWorldEnu_m_s[3];
  input Real initialQuaternionWorldBody[4];
  input Real initialGyroscopeBiasBodyFlu_rad_s[3];
  input Real initialAccelerometerBiasBodyFlu_m_s2[3];
  input Estimation.StrapdownINS.InitialVariances initialVariances;
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  input Real innovationGate;
  input Real opticalFlowGroundNormalWorldEnu[3];
  input Real opticalFlowGroundPlaneOffset_m;
  input Integer mocapRejectionsPrevious;
  input Integer gpsRejectionsPrevious;
  input Integer opticalFlowRejectionsPrevious;
  output Real positionWorldEnu_m[3];
  output Real velocityWorldEnu_m_s[3];
  output Real quaternionWorldBody[4];
  output Real gyroscopeBiasBodyFlu_rad_s[3];
  output Real accelerometerBiasBodyFlu_m_s2[3];
  output Real covariance[TangentLength, TangentLength];
  output Boolean initialized;
  output Boolean predictionAccepted;
  output Boolean mocapCorrectionAccepted;
  output Boolean gpsCorrectionAccepted;
  output Boolean opticalFlowCorrectionAccepted;
  output Integer correctionOutcome;
  output Integer correctionSource;
  output Real normalizedInnovationSquared;
  output Integer mocapRejections;
  output Integer gpsRejections;
  output Integer opticalFlowRejections;
protected
  Estimation.StrapdownINS.UKF.State working;
  Estimation.StrapdownINS.UKF.State corrected;
  Boolean correctionAccepted;
algorithm
  predictionAccepted := false;
  correctionAccepted := false;
  correctionOutcome := Estimation.StrapdownINS.CorrectionNotAttempted;
  correctionSource := Estimation.StrapdownINS.SourceNone;
  normalizedInnovationSquared := 0.0;

  if reset or not initializedPrevious then
    working := initialize(
      initialPositionWorldEnu_m,
      initialVelocityWorldEnu_m_s,
      initialQuaternionWorldBody,
      initialGyroscopeBiasBodyFlu_rad_s,
      initialAccelerometerBiasBodyFlu_m_s2,
      initialVariances);
    initialized := true;
  elseif imu.valid and imu.fresh then
    (working, predictionAccepted) := predict(
      previous,
      imu.angularVelocityBodyFlu_rad_s,
      imu.specificForceBodyFlu_m_s2,
      gravityWorldEnu_m_s2,
      dt,
      processNoise);
    if not predictionAccepted then
      // A failed sigma-point factorization must not publish a partially
      // constructed prediction. Preserve the last valid navigation state and
      // let common health telemetry expose the rejected prediction.
      working := Estimation.StrapdownINS.UKF.State(
        positionWorldEnu_m=previous.positionWorldEnu_m,
        velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
        quaternionWorldBody=previous.quaternionWorldBody,
        gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
        accelerometerBiasBodyFlu_m_s2=
          previous.accelerometerBiasBodyFlu_m_s2,
        covariance=previous.covariance);
    end if;
    initialized := initializedPrevious;
  else
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=previous.positionWorldEnu_m,
      velocityWorldEnu_m_s=previous.velocityWorldEnu_m_s,
      quaternionWorldBody=previous.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=previous.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        previous.accelerometerBiasBodyFlu_m_s2,
      covariance=previous.covariance);
    initialized := initializedPrevious;
  end if;

  if initialized and mocap.valid and mocap.fresh then
    correctionSource := Estimation.StrapdownINS.SourceMocap;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctMocap(
        working, mocap, innovationGate);
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
  elseif initialized and gps.valid and gps.fresh
      and gps.positionValid and gps.velocityValid then
    correctionSource := Estimation.StrapdownINS.SourceGps;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctGps(
        working, gps, innovationGate);
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
  elseif initialized and opticalFlow.valid and opticalFlow.fresh then
    correctionSource := Estimation.StrapdownINS.SourceOpticalFlow;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctOpticalFlow(
        working, opticalFlow, innovationGate,
        opticalFlowGroundNormalWorldEnu,
        opticalFlowGroundPlaneOffset_m);
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
  end if;

  mocapCorrectionAccepted := correctionAccepted and
    correctionSource == Estimation.StrapdownINS.SourceMocap;
  gpsCorrectionAccepted := correctionAccepted and
    correctionSource == Estimation.StrapdownINS.SourceGps;
  opticalFlowCorrectionAccepted := correctionAccepted and
    correctionSource == Estimation.StrapdownINS.SourceOpticalFlow;
  mocapRejections := if correctionSource
      == Estimation.StrapdownINS.SourceMocap then
    (if correctionAccepted then 0 else mocapRejectionsPrevious + 1)
    else mocapRejectionsPrevious;
  gpsRejections := if correctionSource
      == Estimation.StrapdownINS.SourceGps then
    (if correctionAccepted then 0 else gpsRejectionsPrevious + 1)
    else gpsRejectionsPrevious;
  opticalFlowRejections := if correctionSource
      == Estimation.StrapdownINS.SourceOpticalFlow then
    (if correctionAccepted then 0 else opticalFlowRejectionsPrevious + 1)
    else opticalFlowRejectionsPrevious;

  positionWorldEnu_m := working.positionWorldEnu_m;
  velocityWorldEnu_m_s := working.velocityWorldEnu_m_s;
  quaternionWorldBody := working.quaternionWorldBody;
  gyroscopeBiasBodyFlu_rad_s := working.gyroscopeBiasBodyFlu_rad_s;
  accelerometerBiasBodyFlu_m_s2 :=
    working.accelerometerBiasBodyFlu_m_s2;
  covariance := working.covariance;
end step;
