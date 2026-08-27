within Estimation.StrapdownINS.UKF;

function step
  "Execute one sampled UKF prediction/correction tick"
  input Boolean initializedPrevious;
  input Estimation.StrapdownINS.UKF.State previous;
  input Boolean reset;
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.MagnetometerSample magnetometer;
  input Avionics.BarometerSample barometer;
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
  input Real localMagneticFieldWorldEnu_T[3];
  input Real barometerBias_m;
  input Real barometerBiasVariance_m2;
  input Real maximumAidingDelay_s;
  input Real minimumOpticalFlowQuality;
  input Real minimumOpticalFlowGroundDistance_m;
  input Integer mocapRejectionsPrevious;
  input Integer gpsRejectionsPrevious;
  input Integer opticalFlowRejectionsPrevious;
  input Real imuTimestampConsumedPrevious_s;
  input Real mocapTimestampConsumedPrevious_s;
  input Real gpsTimestampConsumedPrevious_s;
  input Real magnetometerTimestampConsumedPrevious_s;
  input Real barometerTimestampConsumedPrevious_s;
  input Real opticalFlowTimestampConsumedPrevious_s;
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
  output Boolean magnetometerCorrectionAccepted;
  output Boolean barometerCorrectionAccepted;
  output Boolean opticalFlowCorrectionAccepted;
  output Integer correctionOutcome;
  output Integer correctionSource;
  output Real normalizedInnovationSquared;
  output Integer mocapRejections;
  output Integer gpsRejections;
  output Integer opticalFlowRejections;
  output Real imuTimestampConsumed_s;
  output Real mocapTimestampConsumed_s;
  output Real gpsTimestampConsumed_s;
  output Real magnetometerTimestampConsumed_s;
  output Real barometerTimestampConsumed_s;
  output Real opticalFlowTimestampConsumed_s;
protected
  Estimation.StrapdownINS.UKF.State working;
  Estimation.StrapdownINS.UKF.State corrected;
  Boolean correctionAccepted;
  Boolean alignmentAccepted;
  Boolean imuNew;
  Boolean mocapNew;
  Boolean gpsNew;
  Boolean magnetometerNew;
  Boolean barometerNew;
  Boolean opticalFlowNew;
  Boolean imuPayloadFinite;
  Boolean mocapSeedUsable;
  Boolean gpsSeedUsable;
  Boolean magnetometerSeedUsable;
  Real mocapSeedQuaternionNorm;
  Real initializationPosition[3];
  Real initializationQuaternion[4];
algorithm
  predictionAccepted := false;
  correctionAccepted := false;
  correctionOutcome := Estimation.StrapdownINS.CorrectionNotAttempted;
  correctionSource := Estimation.StrapdownINS.SourceNone;
  normalizedInnovationSquared := 0.0;
  magnetometerCorrectionAccepted := false;
  barometerCorrectionAccepted := false;
  imuTimestampConsumed_s := imuTimestampConsumedPrevious_s;
  mocapTimestampConsumed_s := mocapTimestampConsumedPrevious_s;
  gpsTimestampConsumed_s := gpsTimestampConsumedPrevious_s;
  magnetometerTimestampConsumed_s := magnetometerTimestampConsumedPrevious_s;
  barometerTimestampConsumed_s := barometerTimestampConsumedPrevious_s;
  opticalFlowTimestampConsumed_s := opticalFlowTimestampConsumedPrevious_s;
  // AFFIRMATIVE ADMISSION, matching the ESKF. `valid` is a claim by the
  // driver; finiteness is a property of the numbers, and only the second one
  // protects the prediction path, which has no innovation gate in front of it.
  // A sample that fails here is treated exactly as a missing one.
  imuPayloadFinite := Estimation.StrapdownINS.imuSampleFinite(imu);
  imuNew := imu.valid
    and imuPayloadFinite
    and imu.timestamp_s > imuTimestampConsumedPrevious_s + 1.0e-9;
  mocapNew := mocap.valid
    and abs(mocap.timestamp_s) < ESKF.FiniteMagnitudeLimit
    and mocap.timestamp_s > mocapTimestampConsumedPrevious_s + 1.0e-9;
  gpsNew := gps.valid
    and abs(gps.timestamp_s) < ESKF.FiniteMagnitudeLimit
    and gps.timestamp_s > gpsTimestampConsumedPrevious_s + 1.0e-9;
  magnetometerNew := magnetometer.valid
    and abs(magnetometer.timestamp_s) < ESKF.FiniteMagnitudeLimit
    and magnetometer.timestamp_s
      > magnetometerTimestampConsumedPrevious_s + 1.0e-9;
  barometerNew := barometer.valid
    and abs(barometer.timestamp_s) < ESKF.FiniteMagnitudeLimit
    and barometer.timestamp_s > barometerTimestampConsumedPrevious_s + 1.0e-9;
  opticalFlowNew := opticalFlow.valid
    and abs(opticalFlow.timestamp_s) < ESKF.FiniteMagnitudeLimit
    and opticalFlow.timestamp_s
      > opticalFlowTimestampConsumedPrevious_s + 1.0e-9;

  // Seed admission. initialize() normalizes whatever attitude it is handed, so
  // a seed that cannot prove it is normalizable would become a full-scale
  // rotation the filter can never be argued out of.
  mocapSeedUsable := mocap.valid;
  mocapSeedQuaternionNorm := 0.0;
  for i in 1:3 loop
    mocapSeedUsable := mocapSeedUsable
      and abs(mocap.positionWorldEnu_m[i]) < ESKF.FiniteMagnitudeLimit;
  end for;
  for i in 1:4 loop
    mocapSeedUsable := mocapSeedUsable
      and abs(mocap.quaternionWorldBody[i]) < ESKF.FiniteMagnitudeLimit;
    mocapSeedQuaternionNorm := mocapSeedQuaternionNorm
      + mocap.quaternionWorldBody[i] * mocap.quaternionWorldBody[i];
  end for;
  mocapSeedUsable := mocapSeedUsable
    and sqrt(mocapSeedQuaternionNorm) >= ESKF.MinimumSeedQuaternionNorm;
  gpsSeedUsable := gps.valid and gps.positionValid;
  for i in 1:3 loop
    gpsSeedUsable := gpsSeedUsable
      and abs(gps.positionWorldEnu_m[i]) < ESKF.FiniteMagnitudeLimit;
  end for;
  magnetometerSeedUsable := magnetometer.valid;
  for i in 1:3 loop
    magnetometerSeedUsable := magnetometerSeedUsable
      and abs(magnetometer.magneticFieldBodyFlu_T[i])
        < ESKF.FiniteMagnitudeLimit
      and abs(localMagneticFieldWorldEnu_T[i]) < ESKF.FiniteMagnitudeLimit
      and magnetometer.covarianceBody_T2[i, i] > 0.0;
  end for;

  if reset or not initializedPrevious then
    initializationPosition := if gpsSeedUsable then
      gps.positionWorldEnu_m else initialPositionWorldEnu_m;
    if mocapSeedUsable then
      initializationQuaternion := mocap.quaternionWorldBody;
      alignmentAccepted := true;
    elseif imu.valid and imuPayloadFinite and magnetometerSeedUsable then
      (initializationQuaternion, alignmentAccepted) :=
        Estimation.StrapdownINS.initialAlignmentQuaternion(
          imu.specificForceBodyFlu_m_s2,
          magnetometer.magneticFieldBodyFlu_T,
          localMagneticFieldWorldEnu_T,
          initialQuaternionWorldBody);
    else
      initializationQuaternion := initialQuaternionWorldBody;
      alignmentAccepted := false;
    end if;
    working := initialize(
      initializationPosition,
      initialVelocityWorldEnu_m_s,
      initializationQuaternion,
      initialGyroscopeBiasBodyFlu_rad_s,
      initialAccelerometerBiasBodyFlu_m_s2,
      initialVariances);
    initialized := true;
    magnetometerCorrectionAccepted := magnetometerSeedUsable
      and alignmentAccepted
      and not mocapSeedUsable;
  elseif imuNew then
    (working, predictionAccepted) := predictPreintegrated(
      previous, imu, gravityWorldEnu_m_s2, processNoise);
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
    imuTimestampConsumed_s := imu.timestamp_s;
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

  if initialized and mocapNew then
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
    mocapTimestampConsumed_s := mocap.timestamp_s;
  elseif initialized and gpsNew
      and gps.positionValid and gps.velocityValid then
    correctionSource := Estimation.StrapdownINS.SourceGps;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctGps(
        working, gps, innovationGate,
        imu.timestamp_s - gps.timestamp_s,
        imu.angularVelocityBodyFlu_rad_s,
        imu.specificForceBodyFlu_m_s2, gravityWorldEnu_m_s2,
        maximumAidingDelay_s);
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
    gpsTimestampConsumed_s := gps.timestamp_s;
  elseif initialized and barometerNew then
    correctionSource := Estimation.StrapdownINS.SourceBarometer;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctBarometer(
        working, barometer, barometerBias_m, barometerBiasVariance_m2,
        innovationGate, imu.timestamp_s - barometer.timestamp_s,
        imu.angularVelocityBodyFlu_rad_s,
        imu.specificForceBodyFlu_m_s2, gravityWorldEnu_m_s2,
        maximumAidingDelay_s);
    barometerCorrectionAccepted := correctionAccepted;
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
    barometerTimestampConsumed_s := barometer.timestamp_s;
  elseif initialized and opticalFlowNew then
    correctionSource := Estimation.StrapdownINS.SourceOpticalFlow;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctOpticalFlow(
        working, opticalFlow, innovationGate,
        imu.timestamp_s - opticalFlow.timestamp_s,
        imu.angularVelocityBodyFlu_rad_s,
        imu.specificForceBodyFlu_m_s2, gravityWorldEnu_m_s2,
        maximumAidingDelay_s, minimumOpticalFlowQuality,
        minimumOpticalFlowGroundDistance_m);
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
    opticalFlowTimestampConsumed_s := opticalFlow.timestamp_s;
  elseif initialized and magnetometerNew then
    correctionSource := Estimation.StrapdownINS.SourceMagnetometer;
    (corrected, correctionAccepted, correctionOutcome,
      normalizedInnovationSquared) := correctMagnetometer(
        working, magnetometer, localMagneticFieldWorldEnu_T,
        innovationGate, imu.timestamp_s - magnetometer.timestamp_s,
        imu.angularVelocityBodyFlu_rad_s,
        imu.specificForceBodyFlu_m_s2, gravityWorldEnu_m_s2,
        maximumAidingDelay_s);
    magnetometerCorrectionAccepted := correctionAccepted;
    working := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=corrected.positionWorldEnu_m,
      velocityWorldEnu_m_s=corrected.velocityWorldEnu_m_s,
      quaternionWorldBody=corrected.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=corrected.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        corrected.accelerometerBiasBodyFlu_m_s2,
      covariance=corrected.covariance);
    magnetometerTimestampConsumed_s := magnetometer.timestamp_s;
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
