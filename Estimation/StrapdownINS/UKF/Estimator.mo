within Estimation.StrapdownINS.UKF;

block Estimator
  "Sampled manifold UKF implementing the common strapdown estimator boundary"
  extends Estimation.StrapdownINS.PartialEstimator;

  parameter Real innovationGate = 6.0
    "NIS gate per measurement degree of freedom; non-positive disables";

protected
  discrete Real statePositionWorldEnu_m[3](each start=0.0, each fixed=true);
  discrete Real stateVelocityWorldEnu_m_s[3](each start=0.0, each fixed=true);
  discrete Real stateQuaternionWorldBody[4](
    start={1.0, 0.0, 0.0, 0.0}, each fixed=true);
  discrete Real stateGyroscopeBiasBodyFlu_rad_s[3](
    each start=0.0, each fixed=true);
  discrete Real stateAccelerometerBiasBodyFlu_m_s2[3](
    each start=0.0, each fixed=true);
  discrete Estimation.StrapdownINS.UKF.Covariance stateCovariance(
    each start=0.0, each fixed=true);
  discrete Boolean initialized(start=false, fixed=true);
  discrete Boolean predictionSucceeded(start=false, fixed=true);
  discrete Boolean mocapCorrectionAccepted(start=false, fixed=true);
  discrete Boolean gpsCorrectionAccepted(start=false, fixed=true);
  discrete Boolean magnetometerCorrectionAccepted(start=false, fixed=true);
  discrete Boolean barometerCorrectionAccepted(start=false, fixed=true);
  discrete Boolean opticalFlowCorrectionAccepted(start=false, fixed=true);
  discrete Boolean barometerBiasInitialized(start=false, fixed=true);
  discrete Boolean barometerBiasUpdateAccepted(start=false, fixed=true);
  discrete Integer barometerBiasCalibrationCount(start=0, fixed=true);
  discrete Real stateBarometerBias_m(start=0.0, fixed=true);
  discrete Real stateBarometerBiasVariance_m2(start=4.0, fixed=true);
  discrete Boolean terrainInitialized(start=false, fixed=true);
  discrete Real stateTerrainAltitude_m(start=0.0, fixed=true);
  discrete Real stateTerrainVariance_m2(start=4.0, fixed=true);
  discrete Boolean terrainCorrectionAccepted(start=false, fixed=true);
  discrete Real auxiliaryRotationWorldBody[3, 3](each start=0.0, each fixed=true);
  discrete Real auxiliaryPredictedVariance_m2(start=1.0, fixed=true);
  discrete Real auxiliaryObservationVariance_m2(start=1.0, fixed=true);
  discrete Real auxiliaryInnovationVariance_m2(start=1.0, fixed=true);
  discrete Real auxiliaryGain(start=0.0, fixed=true);
  discrete Real auxiliaryObservation_m(start=0.0, fixed=true);
  discrete Real auxiliaryCosTilt(start=1.0, fixed=true);
  discrete Integer mocapRejections(start=0, fixed=true);
  discrete Integer gpsRejections(start=0, fixed=true);
  discrete Integer opticalFlowRejections(start=0, fixed=true);
  discrete Integer correctionReason(start=0, fixed=true);
  discrete Integer acceptedCorrectionCount(start=0, fixed=true)
    "Monotonic count of SHIFTED FUSION INSTANTS on the status boundary: ticks
     on which at least one aiding correction was accepted, at most one per
     tick. See Avionics.EstimatorStatus.acceptedCorrectionCount, which states
     the contract; this filter satisfies it by construction because its
     correction dispatch is a priority chain that fuses at most one source per
     tick, so correctionOutcome carries a single value per tick and this
     increments on it.

     The outcome field is a level held for the whole filter tick, so a
     consumer running faster than the filter cannot edge-detect it; the count
     gives that consumer a well-defined edge.";
  discrete Integer correctionSource(start=0, fixed=true);
  discrete Real correctionNis(start=0.0, fixed=true);
  discrete Real imuTimestampConsumed_s(start=-1.0e30, fixed=true);
  discrete Real mocapTimestampConsumed_s(start=-1.0e30, fixed=true);
  discrete Real gpsTimestampConsumed_s(start=-1.0e30, fixed=true);
  discrete Real magnetometerTimestampConsumed_s(start=-1.0e30, fixed=true);
  discrete Real barometerTimestampConsumed_s(start=-1.0e30, fixed=true);
  discrete Real opticalFlowTimestampConsumed_s(start=-1.0e30, fixed=true);
  discrete Real barometerBiasTimestampConsumed_s(
    start=-1.0e30, fixed=true);
  discrete Real terrainTimestampConsumed_s(start=-1.0e30, fixed=true);

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
      pre(stateQuaternionWorldBody));
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
    auxiliaryObservation_m := pre(statePositionWorldEnu_m[3])
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
    (statePositionWorldEnu_m,
     stateVelocityWorldEnu_m_s,
     stateQuaternionWorldBody,
     stateGyroscopeBiasBodyFlu_rad_s,
     stateAccelerometerBiasBodyFlu_m_s2,
     stateCovariance,
     initialized,
     predictionSucceeded,
     mocapCorrectionAccepted,
     gpsCorrectionAccepted,
     magnetometerCorrectionAccepted,
     barometerCorrectionAccepted,
     opticalFlowCorrectionAccepted,
     correctionReason,
     correctionSource,
     correctionNis,
     mocapRejections,
     gpsRejections,
     opticalFlowRejections,
     imuTimestampConsumed_s,
     mocapTimestampConsumed_s,
     gpsTimestampConsumed_s,
     magnetometerTimestampConsumed_s,
     barometerTimestampConsumed_s,
     opticalFlowTimestampConsumed_s) := Estimation.StrapdownINS.UKF.step(
       pre(initialized),
       Estimation.StrapdownINS.UKF.State(
         positionWorldEnu_m=pre(statePositionWorldEnu_m),
         velocityWorldEnu_m_s=pre(stateVelocityWorldEnu_m_s),
         quaternionWorldBody=pre(stateQuaternionWorldBody),
         gyroscopeBiasBodyFlu_rad_s=pre(stateGyroscopeBiasBodyFlu_rad_s),
         accelerometerBiasBodyFlu_m_s2=
           pre(stateAccelerometerBiasBodyFlu_m_s2),
         covariance=pre(stateCovariance)),
       reset, imu, mocap, gps, magnetometer,
       Avionics.BarometerSample(
         valid=barometer.valid and pre(barometerBiasInitialized)
           and barometer.timestamp_s
             > pre(barometerBiasTimestampConsumed_s) + 1.0e-9,
         fresh=barometer.fresh,
         timestamp_s=barometer.timestamp_s,
         altitudeWorldEnu_m=barometer.altitudeWorldEnu_m,
         variance_m2=barometer.variance_m2),
       opticalFlow,
       gravityWorldEnu_m_s2, samplePeriod,
       initialPositionWorldEnu_m, initialVelocityWorldEnu_m_s,
       initialQuaternionWorldBody, initialGyroscopeBiasBodyFlu_rad_s,
       initialAccelerometerBiasBodyFlu_m_s2, initialVariances,
       processNoise, innovationGate,
       localMagneticFieldWorldEnu_T,
       stateBarometerBias_m, stateBarometerBiasVariance_m2,
       maximumAidingDelay_s, minimumOpticalFlowQuality,
       minimumOpticalFlowGroundDistance_m,
       pre(mocapRejections), pre(gpsRejections),
       pre(opticalFlowRejections),
       pre(imuTimestampConsumed_s),
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
          positionWorldEnu_m=statePositionWorldEnu_m,
          velocityWorldEnu_m_s=stateVelocityWorldEnu_m_s,
          quaternionWorldBody=stateQuaternionWorldBody,
          gyroscopeBiasBodyFlu_rad_s=stateGyroscopeBiasBodyFlu_rad_s,
          accelerometerBiasBodyFlu_m_s2=
            stateAccelerometerBiasBodyFlu_m_s2,
          covariance=stateCovariance),
        imu, gravityWorldEnu_m_s2, initialized);

    acceptedCorrectionCount := if reset then 0
      elseif correctionReason == Estimation.StrapdownINS.CorrectionAccepted
        then pre(acceptedCorrectionCount) + 1
      else pre(acceptedCorrectionCount);
    status.initialized := initialized;
    status.predictionAccepted := predictionSucceeded;
    status.mocapCorrectionAccepted := mocapCorrectionAccepted;
    status.gpsPositionCorrectionAccepted := gpsCorrectionAccepted;
    status.gpsVelocityCorrectionAccepted := gpsCorrectionAccepted;
    status.magnetometerCorrectionAccepted := magnetometerCorrectionAccepted;
    status.barometerCorrectionAccepted := barometerCorrectionAccepted;
    status.terrainCorrectionAccepted := terrainCorrectionAccepted;
    status.opticalFlowCorrectionAccepted := opticalFlowCorrectionAccepted;
    status.consecutiveRejectedCorrections :=
      if mocapRejections >= gpsRejections
          and mocapRejections >= opticalFlowRejections then
        mocapRejections
      elseif gpsRejections >= opticalFlowRejections then gpsRejections
      else opticalFlowRejections;
    status.rejectionElapsed_s := samplePeriod
      * status.consecutiveRejectedCorrections;
    status.mocapConsecutiveRejections := mocapRejections;
    status.gpsConsecutiveRejections := gpsRejections;
    status.opticalFlowConsecutiveRejections := opticalFlowRejections;
    status.correctionOutcome := correctionReason;
    status.acceptedCorrectionCount := acceptedCorrectionCount;
    status.correctionSource := correctionSource;
    status.normalizedInnovationSquared := correctionNis;
    status.recoveryStage := Estimation.StrapdownINS.RecoveryNominal;
    status.anchorSource := correctionSource;
    status.imuPayloadHeld := not predictionSucceeded;
  end when;

equation
  navigationCovarianceLocal = stateCovariance[1:6, 1:6];
  gyroscopeBiasBodyFlu_rad_s = stateGyroscopeBiasBodyFlu_rad_s;
  accelerometerBiasBodyFlu_m_s2 = stateAccelerometerBiasBodyFlu_m_s2;
  barometerBias_m = stateBarometerBias_m;
  barometerBiasVariance_m2 = stateBarometerBiasVariance_m2;
  terrainAltitudeWorldEnu_m = stateTerrainAltitude_m;
  terrainAltitudeVariance_m2 = stateTerrainVariance_m2;
  heightAboveTerrain_m = statePositionWorldEnu_m[3] - stateTerrainAltitude_m;
end Estimator;
