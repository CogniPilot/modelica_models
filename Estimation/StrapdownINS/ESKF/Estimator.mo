within Estimation.StrapdownINS.ESKF;

block Estimator
  "Sampled aided strapdown inertial-navigation ESKF"
  extends Estimation.StrapdownINS.PartialEstimator;

  // Consistency instrumentation (NEES/NIS study): mirror the full
  // error-state covariance and the bias estimates as public outputs so
  // a mission-level consistency evaluation can be post-processed from
  // exported channels without touching the filter internals.
  output Real errorCovariance[15, 15](
    each start = 0.0, each fixed = true)
    "Full local-error tangent covariance, mirrored each estimator tick";
  output Real estimatedGyroscopeBias_rad_s[3](
    each start = 0.0, each fixed = true)
    "Gyroscope bias estimate, mirrored each estimator tick";
  output Real estimatedAccelerometerBias_m_s2[3](
    each start = 0.0, each fixed = true)
    "Accelerometer bias estimate, mirrored each estimator tick";

  parameter Estimation.StrapdownINS.ESKF.VarianceLimits varianceLimits =
    Estimation.StrapdownINS.ESKF.VarianceLimits(
      position_m2=fill(1.0e4, 3),
      velocity_m2_s2=fill(4.0e2, 3),
      attitude_rad2=fill(10.0, 3),
      gyroscopeBias_rad2_s2=fill(1.0e-2, 3),
      accelerometerBias_m2_s4=fill(1.0, 3))
    "Diagonal covariance growth bounds from the state units and mission
     envelope: position sigma 100 m dwarfs the sub-100 m RDD2 flight
     volume; velocity sigma 20 m/s is beyond the airframe speed
     envelope; attitude sigma sqrt(10) ~ pi rad means orientation fully
     unknown; gyro-bias sigma 0.1 rad/s and accelerometer-bias sigma
     1 m/s2 (~0.1 g) bound consumer-MEMS turn-on bias. At these values
     the state is already maximally uninformative, so capping there
     loses no information while keeping the covariance inside the
     dynamic range a binary32 Cholesky can factor.";
  parameter Real innovationGate(unit = "1") = 6.0
    "Chi-square innovation gate per degree of freedom; non-positive
     disables. NIS is chi-square with m degrees of freedom for a
     consistent filter; 6.0 per dof keeps the false-rejection
     probability at or below ~0.25% for the m = 2, 3, 6 corrections
     used here (chi-square tails: 12 @ m=2 -> 0.25%, 18 @ m=3 ->
     0.044%, 36 @ m=6 -> 2.8e-6).";
  // The recovery-window parameters are validated both at the public block
  // boundary and inside the pure step function. The duplicated contract is
  // intentional: invalid configuration must disable recovery rather than
  // silently selecting an unsafe recovery mode.
  parameter Real covarianceInflateWindow_s(unit = "s") = 1.0
    "Unbroken wall-clock time during which the anchor source has been
     unable to move the state, after which the position and velocity
     covariance begins ramping toward the mission envelope with the state
     left untouched. Long enough that a short outlier burst cannot provoke
     it, short enough that an over-tight covariance is reopened well
     before dead-reckoning drift matters.

     This is a TIME, not a count of rejected corrections. The count it
     replaces was documented as 'aiding streams at 20-100 Hz, so 50
     rejections spans roughly 0.5-2.5 s' -- but nothing tied the count to
     the aiding rate, and at the 1 kHz rate this block is actually wired
     at, with aiding asserted every tick, 50 rejections was 51 ms:
     10-50x faster than the latency the value was chosen for. It is
     advanced on estimator ticks while the anchor holds without accepting,
     so it measures elapsed time at every wiring and aiding rate, and in
     particular under the pulsed sensor wiring the deployed GPS driver
     actually produces.";
  parameter Real covarianceInflateTimeConstant_s(unit = "s")
      = 0.5
    "e-folding time of the covariance ramp once the window has elapsed.

     The ramp exists because the alternative -- setting the covariance
     straight to the mission envelope -- converts gating into adoption:
     at envelope variance against a typical reported measurement noise the
     Kalman gain is 0.9998, so the next sample simply becomes the state.
     Measured with that jump in place, and strictly worse than taking no
     action at all: a cold-start filter against a truthful 8-12 m/s stream
     acquired tens of m/s of spurious velocity and +52 m of position
     error, and the discrepancy at which a lying sole optical-flow sensor
     was adopted rather than rejected fell from 3.63 m/s to 0.72 m/s.

     0.5 s doubles a variance about every 0.35 s, so a gate locked out by
     a factor of ten reopens in roughly a second while any single tick can
     widen it by at most a factor of 1.004 at 1 kHz. That bound is the
     point: the gain available immediately after inflation cannot swing
     the state outside the pre-inflation gate.";
  parameter Real aidingDivergentWindow_s(unit = "s") = 5.0
    "Unbroken wall-clock time after which the anchor is REPORTED
     divergent on status.recoveryStage. Five times the inflate window: if
     ramping the covariance to the full mission envelope has not let the
     anchor back in within that span, the disagreement is larger than the
     envelope the vehicle is declared to operate in.

     This raises a status flag and nothing else. It deliberately does NOT
     withdraw estimate.valid: on the deployed stack that clears
     RatesValid, zeroes the motors and latches the fault in every mode
     including ACRO, so it is a crash rather than the failsafe handover it
     was intended to be. The wrapper maps this stage onto a non-latching
     mode demotion.";
  parameter Real aidingStaleTimeout_s(unit = "s") = 0.5
    "How long a source may go without a fresh sample before it stops
     counting as available for anchor selection.

     Sensor `valid` cannot be read as availability on a single tick. The
     deployed GPS driver pulses `valid` with `fresh`, asserting it only on
     the ticks that carry a fix, so a rule keyed on `valid` sees GPS as
     absent for 99 of every 100 ticks at 10 Hz. The mocap wrapper has the
     opposite property -- it derives `valid` from record contents and
     `fresh` from subscription updates, independently, so a source can be
     permanently valid and never fresh. 0.5 s spans ten intervals of the
     slowest aiding rate this block is specified for while still demoting
     a genuinely stalled source well inside the divergence window.";

protected
  discrete Real statePosition[3](each start = 0.0, each fixed = true);
  discrete Real stateVelocity[3](each start = 0.0, each fixed = true);
  discrete Real stateQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real stateGyroscopeBias[3](each start = 0.0, each fixed = true);
  discrete Real stateAccelerometerBias[3](each start = 0.0, each fixed = true);
  discrete Real stateCovariance[15, 15](each start = 0.0, each fixed = true);
  discrete Boolean initialized(start = false, fixed = true);
  discrete Boolean predictionAccepted(start = false, fixed = true);
  discrete Boolean mocapCorrectionAccepted(start = false, fixed = true);
  discrete Boolean gpsPositionCorrectionAccepted(start = false, fixed = true);
  discrete Boolean gpsVelocityCorrectionAccepted(start = false, fixed = true);
  discrete Boolean magnetometerCorrectionAccepted(start = false, fixed = true);
  discrete Boolean barometerCorrectionAccepted(start = false, fixed = true);
  discrete Boolean opticalFlowCorrectionAccepted(start = false, fixed = true);
  discrete Boolean barometerBiasInitialized(start = false, fixed = true);
  discrete Boolean barometerBiasUpdateAccepted(start = false, fixed = true);
  discrete Integer barometerBiasCalibrationCount(start = 0, fixed = true);
  discrete Real stateBarometerBias_m(start = 0.0, fixed = true);
  discrete Real stateBarometerBiasVariance_m2(start = 4.0, fixed = true);
  discrete Boolean terrainInitialized(start = false, fixed = true);
  discrete Real stateTerrainAltitude_m(start = 0.0, fixed = true);
  discrete Real stateTerrainVariance_m2(start = 4.0, fixed = true);
  discrete Boolean terrainCorrectionAccepted(start = false, fixed = true);
  discrete Real auxiliaryRotationWorldBody[3, 3](each start=0.0, each fixed=true);
  discrete Real auxiliaryPredictedVariance_m2(start=1.0, fixed=true);
  discrete Real auxiliaryObservationVariance_m2(start=1.0, fixed=true);
  discrete Real auxiliaryInnovationVariance_m2(start=1.0, fixed=true);
  discrete Real auxiliaryGain(start=0.0, fixed=true);
  discrete Real auxiliaryObservation_m(start=0.0, fixed=true);
  discrete Real auxiliaryCosTilt(start=1.0, fixed=true);
  discrete Integer consecutiveRejectedCorrections(start = 0, fixed = true);
  discrete Real rejectionElapsed_s(start = 0.0, fixed = true);
  discrete Integer mocapRejections(start = 0, fixed = true);
  discrete Integer gpsRejections(start = 0, fixed = true);
  discrete Integer opticalFlowRejections(start = 0, fixed = true);
  discrete Integer recoveryStage(start = 0, fixed = true);
  discrete Integer correctionOutcome(start = 0, fixed = true);
  discrete Integer correctionSource(start = 0, fixed = true);
  discrete Real normalizedInnovationSquared(start = 0.0, fixed = true);
  discrete Boolean estimateValid(start = false, fixed = true);
  discrete Integer anchorSource(start = 0, fixed = true);
  discrete Real mocapStale_s(start = 1.0e30, fixed = true);
  discrete Real gpsStale_s(start = 1.0e30, fixed = true);
  discrete Real opticalFlowStale_s(start = 1.0e30, fixed = true);
  discrete Real imuAngularVelocityHeld_rad_s[3](each start = 0.0, each fixed = true);
  discrete Real imuSpecificForceHeld_m_s2[3](each start = 0.0, each fixed = true);
  discrete Real imuTimestampHeld_s(start = 0.0, fixed = true);
  discrete Boolean imuPayloadHeld(start = false, fixed = true);
  discrete Real mocapTimestampConsumed_s(start = -1.0e30, fixed = true);
  discrete Real gpsTimestampConsumed_s(start = -1.0e30, fixed = true);
  discrete Real magnetometerTimestampConsumed_s(start = -1.0e30, fixed = true);
  discrete Real barometerTimestampConsumed_s(start = -1.0e30, fixed = true);
  discrete Real opticalFlowTimestampConsumed_s(start = -1.0e30, fixed = true);
  discrete Real barometerBiasTimestampConsumed_s(
    start = -1.0e30, fixed = true);
  discrete Real terrainTimestampConsumed_s(start = -1.0e30, fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    barometerBiasTimestampConsumed_s := if not reset
        and barometer.valid
        and abs(barometer.timestamp_s)
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
        and barometer.timestamp_s
          > pre(barometerBiasTimestampConsumed_s) + 1.0e-9
      then barometer.timestamp_s
      elseif reset then -1.0e30
      else pre(barometerBiasTimestampConsumed_s);
    terrainTimestampConsumed_s := if not reset
        and opticalFlow.valid
        and abs(opticalFlow.timestamp_s)
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
        and opticalFlow.timestamp_s
          > pre(terrainTimestampConsumed_s) + 1.0e-9
      then opticalFlow.timestamp_s
      elseif reset then -1.0e30
      else pre(terrainTimestampConsumed_s);
    auxiliaryRotationWorldBody := LieGroups.SO3.Quat.to_DCM(
      pre(stateQuaternion));
    auxiliaryPredictedVariance_m2 := max(
      if not reset and pre(barometerBiasCalibrationCount) > 0 then
        pre(stateBarometerBiasVariance_m2)
      else initialBarometerBiasVariance_m2, 1.0e-12)
      + max(barometerBiasProcessNoise_m2_s, 0.0) * samplePeriod;
    auxiliaryObservationVariance_m2 := barometer.variance_m2;
    auxiliaryInnovationVariance_m2 := auxiliaryPredictedVariance_m2
      + auxiliaryObservationVariance_m2;
    auxiliaryObservation_m := barometer.altitudeWorldEnu_m
      - initialPositionWorldEnu_m[3];
    // Average a finite stationary startup window against the declared local
    // origin. During this window the sample is withheld from the navigation
    // correction, avoiding the positive feedback caused by using the same
    // pressure observation to update both altitude and its datum.
    if not reset and not pre(barometerBiasInitialized) and barometer.valid
        and abs(barometer.timestamp_s)
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
        and barometer.timestamp_s
          > pre(barometerBiasTimestampConsumed_s) + 1.0e-9
        and barometer.variance_m2 > 0.0
        and auxiliaryInnovationVariance_m2 > 1.0e-12 then
      auxiliaryGain := auxiliaryPredictedVariance_m2
        / auxiliaryInnovationVariance_m2;
      stateBarometerBias_m := (if not reset
          and pre(barometerBiasCalibrationCount) > 0
          then pre(stateBarometerBias_m)
        else initialBarometerBias_m) + auxiliaryGain
          * (auxiliaryObservation_m - (if not reset
              and pre(barometerBiasCalibrationCount) > 0
              then pre(stateBarometerBias_m)
            else initialBarometerBias_m));
      stateBarometerBiasVariance_m2 := max((1.0 - auxiliaryGain)
        * auxiliaryPredictedVariance_m2, 1.0e-12);
      barometerBiasCalibrationCount :=
        pre(barometerBiasCalibrationCount) + 1;
      barometerBiasInitialized := pre(barometerBiasCalibrationCount) + 1
        >= barometerBiasCalibrationSamples;
      barometerBiasUpdateAccepted := true;
    else
      stateBarometerBias_m := if not reset
          and pre(barometerBiasCalibrationCount) > 0
        then pre(stateBarometerBias_m) else initialBarometerBias_m;
      stateBarometerBiasVariance_m2 := auxiliaryPredictedVariance_m2;
      barometerBiasInitialized := not reset and pre(barometerBiasInitialized);
      barometerBiasCalibrationCount := if reset then 0
        else pre(barometerBiasCalibrationCount);
      barometerBiasUpdateAccepted := false;
    end if;

    auxiliaryCosTilt := auxiliaryRotationWorldBody[3, 3];
    auxiliaryPredictedVariance_m2 := max(
      if not reset and pre(terrainInitialized) then
        pre(stateTerrainVariance_m2) else initialTerrainVariance_m2,
      1.0e-12) + max(terrainProcessNoise_m2_s, 0.0) * samplePeriod;
    auxiliaryObservation_m := pre(statePosition[3])
      - opticalFlow.groundDistance_m * auxiliaryCosTilt;
    auxiliaryObservationVariance_m2 := auxiliaryCosTilt * auxiliaryCosTilt
        * opticalFlow.groundDistanceVariance_m2
      + pre(stateCovariance[1, 1]) + pre(stateCovariance[2, 2])
      + pre(stateCovariance[3, 3])
      + opticalFlow.groundDistance_m * opticalFlow.groundDistance_m
        * (pre(stateCovariance[7, 7]) + pre(stateCovariance[8, 8]));
    auxiliaryInnovationVariance_m2 := auxiliaryPredictedVariance_m2
      + auxiliaryObservationVariance_m2;
    if not reset and opticalFlow.valid
        and abs(opticalFlow.timestamp_s)
          < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit
        and opticalFlow.timestamp_s
          > pre(terrainTimestampConsumed_s) + 1.0e-9
        and opticalFlow.quality >= minimumOpticalFlowQuality
        and opticalFlow.groundDistance_m > 0.05
        and opticalFlow.groundDistanceVariance_m2 > 0.0
        and auxiliaryCosTilt >= minimumRangeCosTilt
        and auxiliaryInnovationVariance_m2 > 1.0e-12 then
      auxiliaryGain := auxiliaryPredictedVariance_m2
        / auxiliaryInnovationVariance_m2;
      stateTerrainAltitude_m := (if not reset and pre(terrainInitialized)
          then pre(stateTerrainAltitude_m)
        else initialTerrainAltitudeWorldEnu_m) + auxiliaryGain
          * (auxiliaryObservation_m - (if not reset and pre(terrainInitialized)
              then pre(stateTerrainAltitude_m)
            else initialTerrainAltitudeWorldEnu_m));
      stateTerrainVariance_m2 := max((1.0 - auxiliaryGain)
        * auxiliaryPredictedVariance_m2, 1.0e-12);
      terrainInitialized := true;
      terrainCorrectionAccepted := true;
    else
      stateTerrainAltitude_m := if not reset and pre(terrainInitialized)
        then pre(stateTerrainAltitude_m) else initialTerrainAltitudeWorldEnu_m;
      stateTerrainVariance_m2 := auxiliaryPredictedVariance_m2;
      terrainInitialized := not reset and pre(terrainInitialized);
      terrainCorrectionAccepted := false;
    end if;
    (statePosition,
     stateVelocity,
     stateQuaternion,
     stateGyroscopeBias,
     stateAccelerometerBias,
     stateCovariance,
     initialized,
     predictionAccepted,
     mocapCorrectionAccepted,
     gpsPositionCorrectionAccepted,
     gpsVelocityCorrectionAccepted,
     magnetometerCorrectionAccepted,
     barometerCorrectionAccepted,
     opticalFlowCorrectionAccepted,
     consecutiveRejectedCorrections,
     rejectionElapsed_s,
     recoveryStage,
     correctionOutcome,
     correctionSource,
     normalizedInnovationSquared,
     estimateValid,
     mocapRejections,
     gpsRejections,
     opticalFlowRejections,
     anchorSource,
     mocapStale_s,
     gpsStale_s,
     opticalFlowStale_s,
     imuAngularVelocityHeld_rad_s,
     imuSpecificForceHeld_m_s2,
     imuTimestampHeld_s,
     imuPayloadHeld,
     mocapTimestampConsumed_s,
     gpsTimestampConsumed_s,
     magnetometerTimestampConsumed_s,
     barometerTimestampConsumed_s,
     opticalFlowTimestampConsumed_s) :=
      Estimation.StrapdownINS.ESKF.step(
        pre(initialized),
        Estimation.StrapdownINS.ESKF.State(
          positionWorldEnu_m=pre(statePosition),
          velocityWorldEnu_m_s=pre(stateVelocity),
          quaternionWorldBody=pre(stateQuaternion),
          gyroscopeBiasBodyFlu_rad_s=pre(stateGyroscopeBias),
          accelerometerBiasBodyFlu_m_s2=pre(stateAccelerometerBias),
          covariance=pre(stateCovariance)),
        reset,
        imu,
        mocap,
        gps,
        magnetometer,
        Avionics.BarometerSample(
          valid=barometer.valid and pre(barometerBiasInitialized)
            and barometer.timestamp_s
              > pre(barometerBiasTimestampConsumed_s) + 1.0e-9,
          fresh=barometer.fresh,
          timestamp_s=barometer.timestamp_s,
          altitudeWorldEnu_m=barometer.altitudeWorldEnu_m,
          variance_m2=barometer.variance_m2),
        opticalFlow,
        gravityWorldEnu_m_s2,
        samplePeriod,
        Estimation.StrapdownINS.ESKF.Tuning(
          initialState=Estimation.StrapdownINS.ESKF.NominalState(
            positionWorldEnu_m=initialPositionWorldEnu_m,
            velocityWorldEnu_m_s=initialVelocityWorldEnu_m_s,
            quaternionWorldBody=initialQuaternionWorldBody,
            gyroscopeBiasBodyFlu_rad_s=initialGyroscopeBiasBodyFlu_rad_s,
            accelerometerBiasBodyFlu_m_s2=
              initialAccelerometerBiasBodyFlu_m_s2),
          initialVariances=initialVariances,
          processNoise=processNoise,
          varianceLimits=varianceLimits,
          innovationGate=innovationGate,
          localMagneticFieldWorldEnu_T=localMagneticFieldWorldEnu_T,
          barometerBias_m=stateBarometerBias_m,
          barometerBiasVariance_m2=stateBarometerBiasVariance_m2,
          maximumAidingDelay_s=maximumAidingDelay_s,
          minimumOpticalFlowQuality=minimumOpticalFlowQuality,
          minimumOpticalFlowGroundDistance_m=
            minimumOpticalFlowGroundDistance_m,
          covarianceInflateWindow_s=covarianceInflateWindow_s,
          covarianceInflateTimeConstant_s=covarianceInflateTimeConstant_s,
          aidingDivergentWindow_s=aidingDivergentWindow_s,
          aidingStaleTimeout_s=aidingStaleTimeout_s),
        pre(consecutiveRejectedCorrections),
        pre(rejectionElapsed_s),
        pre(mocapRejections),
        pre(gpsRejections),
        pre(opticalFlowRejections),
        pre(anchorSource),
        pre(mocapStale_s),
        pre(gpsStale_s),
        pre(opticalFlowStale_s),
        pre(imuAngularVelocityHeld_rad_s),
        pre(imuSpecificForceHeld_m_s2),
        pre(imuTimestampHeld_s),
        pre(mocapTimestampConsumed_s),
        pre(gpsTimestampConsumed_s),
        pre(magnetometerTimestampConsumed_s),
        pre(barometerTimestampConsumed_s),
        pre(opticalFlowTimestampConsumed_s));
    (estimate.valid,
     estimate.timestamp_s,
     estimate.positionWorldEnu_m,
     estimate.velocityWorldEnu_m_s,
     estimate.accelerationWorldEnu_m_s2,
     estimate.quaternionWorldBody,
     estimate.rotationWorldBody,
     estimate.eulerRpy_rad,
     estimate.angularVelocityBodyFlu_rad_s,
     estimate.angularVelocityWorldEnu_rad_s) :=
      Estimation.StrapdownINS.ESKF.navigationEstimate(
        Estimation.StrapdownINS.ESKF.State(
          positionWorldEnu_m=statePosition,
          velocityWorldEnu_m_s=stateVelocity,
          quaternionWorldBody=stateQuaternion,
          gyroscopeBiasBodyFlu_rad_s=stateGyroscopeBias,
          accelerometerBiasBodyFlu_m_s2=stateAccelerometerBias,
          covariance=stateCovariance),
        // The HELD sample, never the raw connector: three published fields
        // are computed from the IMU rather than from the state, so passing
        // the raw sample here is what let a non-finite reading reach the
        // payload while prediction was correctly refusing it.
        Avionics.ImuSample(
          valid=imu.valid,
          fresh=imu.fresh,
          timestamp_s=imuTimestampHeld_s,
          angularVelocityBodyFlu_rad_s=imuAngularVelocityHeld_rad_s,
          specificForceBodyFlu_m_s2=imuSpecificForceHeld_m_s2,
          deltaAngleBodyFlu_rad=imu.deltaAngleBodyFlu_rad,
          deltaVelocityBodyFlu_m_s=imu.deltaVelocityBodyFlu_m_s,
          deltaPositionBodyFlu_m=imu.deltaPositionBodyFlu_m,
          deltaQuaternionBodyFlu=imu.deltaQuaternionBodyFlu,
          gyroscopeBiasLinearizationBodyFlu_rad_s=
            imu.gyroscopeBiasLinearizationBodyFlu_rad_s,
          accelerometerBiasLinearizationBodyFlu_m_s2=
            imu.accelerometerBiasLinearizationBodyFlu_m_s2,
          deltaRotationGyroscopeBiasJacobian_s=
            imu.deltaRotationGyroscopeBiasJacobian_s,
          deltaVelocityGyroscopeBiasJacobian_m=
            imu.deltaVelocityGyroscopeBiasJacobian_m,
          deltaVelocityAccelerometerBiasJacobian_s=
            imu.deltaVelocityAccelerometerBiasJacobian_s,
          deltaPositionGyroscopeBiasJacobian_m_s=
            imu.deltaPositionGyroscopeBiasJacobian_m_s,
          deltaPositionAccelerometerBiasJacobian_s2=
            imu.deltaPositionAccelerometerBiasJacobian_s2,
          integrationTime_s=imu.integrationTime_s),
        gravityWorldEnu_m_s2,
        estimateValid);
    status.initialized := initialized;
    status.predictionAccepted := predictionAccepted;
    status.mocapCorrectionAccepted := mocapCorrectionAccepted;
    status.gpsPositionCorrectionAccepted := gpsPositionCorrectionAccepted;
    status.gpsVelocityCorrectionAccepted := gpsVelocityCorrectionAccepted;
    status.magnetometerCorrectionAccepted := magnetometerCorrectionAccepted;
    status.barometerCorrectionAccepted := barometerCorrectionAccepted;
    status.terrainCorrectionAccepted := terrainCorrectionAccepted;
    status.opticalFlowCorrectionAccepted := opticalFlowCorrectionAccepted;
    status.consecutiveRejectedCorrections := consecutiveRejectedCorrections;
    status.rejectionElapsed_s := rejectionElapsed_s;
    status.mocapConsecutiveRejections := mocapRejections;
    status.gpsConsecutiveRejections := gpsRejections;
    status.opticalFlowConsecutiveRejections := opticalFlowRejections;
    status.correctionOutcome := correctionOutcome;
    status.correctionSource := correctionSource;
    status.normalizedInnovationSquared := normalizedInnovationSquared;
    status.recoveryStage := recoveryStage;
    status.anchorSource := anchorSource;
    status.imuPayloadHeld := imuPayloadHeld;
    errorCovariance := stateCovariance;
    estimatedGyroscopeBias_rad_s := stateGyroscopeBias;
    estimatedAccelerometerBias_m_s2 := stateAccelerometerBias;
  end when;

equation
  navigationCovarianceLocal = stateCovariance[1:6, 1:6];
  gyroscopeBiasBodyFlu_rad_s = stateGyroscopeBias;
  accelerometerBiasBodyFlu_m_s2 = stateAccelerometerBias;
  barometerBias_m = stateBarometerBias_m;
  barometerBiasVariance_m2 = stateBarometerBiasVariance_m2;
  terrainAltitudeWorldEnu_m = stateTerrainAltitude_m;
  terrainAltitudeVariance_m2 = stateTerrainVariance_m2;
  heightAboveTerrain_m = statePosition[3] - stateTerrainAltitude_m;

  // Misconfiguration must be loud, not silently reinterpreted. The `min`
  // attributes above stop a negative value landing on 0.0, which is the
  // one value that arms the ladder permanently; these state the ordering
  // and finiteness the ladder actually depends on. A NaN window would
  // otherwise disable the ladder silently, since every comparison against
  // it is false -- the same predicate-exhaustion defect this whole change
  // exists to remove, and it must not reappear in the tuning surface.
  assert(covarianceInflateWindow_s > 0.0
    and covarianceInflateWindow_s < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit,
    "covarianceInflateWindow_s must be finite and strictly positive");
  assert(covarianceInflateTimeConstant_s > 0.0
    and covarianceInflateTimeConstant_s < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit,
    "covarianceInflateTimeConstant_s must be finite and strictly positive");
  assert(aidingStaleTimeout_s > 0.0
    and aidingStaleTimeout_s < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit,
    "aidingStaleTimeout_s must be finite and strictly positive");
  assert(aidingDivergentWindow_s > covarianceInflateWindow_s
    and aidingDivergentWindow_s < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit,
    "aidingDivergentWindow_s must be finite and exceed covarianceInflateWindow_s");

  annotation(Documentation(info = "<html>
    <p>This block is a concrete implementation of the stable estimator
    contract and can be exported directly as an eFMU. Prediction uses one
    vectorized 15-state covariance, including every cross-axis and bias
    correlation. One fresh aiding packet is accepted per estimator tick with
    deterministic priority mocap, GPS, then optical flow; GPS position and
    velocity are fused jointly when both are valid.</p>
    <p>The SE_2(3) pose/velocity/position part uses geometric propagation,
    injection, and covariance reset. The additive gyroscope and accelerometer
    bias states make the complete augmented system neither strictly invariant
    nor exactly log-linear. It is therefore an aided strapdown INS error-state
    Kalman filter, not a strict invariant-filter construction.</p>
    <p><b>Acceptance is affirmative.</b> A measurement moves the state only
    when it has proven it should: the residual is finite, the innovation
    covariance factors, and the normalized innovation squared is non-negative
    and within the gate. Nothing is accepted merely because no rejection test
    happened to fire, which is what allowed a NaN measurement -- against which
    every comparison is false -- to be applied and to poison the state
    permanently.</p>
    <p><b>The estimator never re-seeds its own state from aiding it is
    rejecting.</b> Sustained rejection drives a two-stage ladder timed in
    wall-clock seconds: first the covariance is inflated to the declared
    mission envelope with the state untouched, which is the only recovery
    action that cannot itself be wrong and which is sufficient whenever
    recovery is possible at all; then, if the aiding still does not fit,
    <code>estimate.valid</code> goes false and the vehicle's failsafe takes
    over. Re-seeding position, attitude, and velocity remains available
    through the commanded <code>reset</code> input, where it is a deliberate
    act rather than a filter's guess.</p>
  </html>"));
end Estimator;
