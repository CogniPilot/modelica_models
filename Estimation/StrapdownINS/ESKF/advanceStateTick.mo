within Estimation.StrapdownINS.ESKF;

function advanceStateTick
  "Advance the state through prediction, recovery and aiding as one bounded stage"
  input State previous;
  input Boolean enabled "The dispatcher enables advancement only after initialization";
  input Avionics.ImuSample imu;
  input Avionics.MocapSample mocap;
  input Avionics.GpsSample gps;
  input Avionics.MagnetometerSample magnetometer;
  input Avionics.BarometerSample barometer;
  input Avionics.OpticalFlowSample opticalFlow;
  input Tuning tuning;
  input Real gravityWorldEnu_m_s2[3];
  input Real dt;
  input Boolean inflateCovarianceNow;
  input Boolean imuUsable;
  input Boolean imuPayloadFinite;
  input Boolean vehicleAtRest;
  input Boolean reseedNow;
  input Boolean anchorExclusive;
  input Boolean mocapNew;
  input Boolean gpsNew;
  input Boolean magnetometerNew;
  input Boolean barometerNew;
  input Boolean opticalFlowNew;
  input Boolean aidingLive;
  input Boolean ladderConfigured;
  input Boolean imuQuiet;
  input Integer anchorPresent;
  input Real rejectionElapsedPrevious_s;
  input Real imuTimestampHeldNext_s;
  input Real imuAngularVelocityHeldNext_rad_s[3];
  input Real imuSpecificForceHeldNext_m_s2[3];
  input Real pseudoPositionHoldPrevious_m[3];
  input Real mocapTimestampConsumedPrevious_s;
  input Real gpsTimestampConsumedPrevious_s;
  input Real magnetometerTimestampConsumedPrevious_s;
  input Real barometerTimestampConsumedPrevious_s;
  input Real opticalFlowTimestampConsumedPrevious_s;
  output State working;
  output Boolean predictionAccepted;
  output Boolean stationaryImuCorrectionAccepted;
  output Boolean mocapCorrectionAccepted;
  output Boolean gpsPositionCorrectionAccepted;
  output Boolean gpsVelocityCorrectionAccepted;
  output Boolean magnetometerCorrectionAccepted;
  output Boolean barometerCorrectionAccepted;
  output Boolean opticalFlowCorrectionAccepted;
  output Boolean correctionAttempted;
  output Boolean correctionAccepted;
  output Integer correctionOutcome;
  output Integer correctionSource;
  output Real normalizedInnovationSquared;
  output Boolean reseeded;
  output Integer reseedSource;
  output Boolean zeroVelocityCorrectionAccepted;
  output Boolean pseudoPositionCorrectionAccepted;
  output Real pseudoPositionHoldNext_m[3];
  output Real mocapTimestampConsumedNext_s;
  output Real gpsTimestampConsumedNext_s;
  output Real magnetometerTimestampConsumedNext_s;
  output Real barometerTimestampConsumedNext_s;
  output Real opticalFlowTimestampConsumedNext_s;
protected
  Real reseedPosition[3];
  Real reseedVelocity[3];
  Boolean absoluteAidingAttempted;
  Boolean opticalFlowAttempted;
  Integer aidingOutcome;
  Real aidingNis;
  Boolean pseudoPositionConfigured;
  Boolean zeroVelocityConfigured;
algorithm
  predictionAccepted := false;
  stationaryImuCorrectionAccepted := false;
  mocapCorrectionAccepted := false;
  gpsPositionCorrectionAccepted := false;
  gpsVelocityCorrectionAccepted := false;
  magnetometerCorrectionAccepted := false;
  barometerCorrectionAccepted := false;
  opticalFlowCorrectionAccepted := false;
  correctionAttempted := false;
  correctionAccepted := false;
  correctionOutcome := CorrectionNotAttempted;
  correctionSource := SourceNone;
  normalizedInnovationSquared := 0.0;
  reseeded := false;
  reseedSource := SourceNone;
  zeroVelocityCorrectionAccepted := false;
  pseudoPositionCorrectionAccepted := false;
  pseudoPositionHoldNext_m := pseudoPositionHoldPrevious_m;
  mocapTimestampConsumedNext_s := mocapTimestampConsumedPrevious_s;
  gpsTimestampConsumedNext_s := gpsTimestampConsumedPrevious_s;
  magnetometerTimestampConsumedNext_s := magnetometerTimestampConsumedPrevious_s;
  barometerTimestampConsumedNext_s := barometerTimestampConsumedPrevious_s;
  opticalFlowTimestampConsumedNext_s := opticalFlowTimestampConsumedPrevious_s;
  aidingOutcome := CorrectionNotAttempted;
  aidingNis := 0.0;
  absoluteAidingAttempted := false;
  opticalFlowAttempted := false;
  reseedPosition := zeros(3);
  reseedVelocity := zeros(3);
  working := copyState(previous);
  if enabled then
    (working, predictionAccepted, stationaryImuCorrectionAccepted) :=
      predictTick(previous, imu, tuning, gravityWorldEnu_m_s2, dt,
        inflateCovarianceNow, imuUsable, imuPayloadFinite, vehicleAtRest);

    if reseedNow then
      reseedSource := anchorPresent;
      if anchorPresent == SourceMocap then
        reseedPosition := mocap.positionWorldEnu_m;
        reseedVelocity := working.velocityWorldEnu_m_s;
      else
        reseedPosition := gps.positionWorldEnu_m;
        reseedVelocity := if gps.velocityValid then
          gps.velocityWorldEnu_m_s else working.velocityWorldEnu_m_s;
      end if;
      // Position and velocity, and only their covariance blocks, are replaced;
      // the attitude and bias states and their variances are carried through
      // untouched. The record assembly lives in reseed() so it does not share
      // a common-subexpression temporary with the prediction-side assemblies.
      working := reseed(working, reseedPosition, reseedVelocity,
        tuning.initialVariances);
      reseeded := true;
    end if;

    absoluteAidingAttempted := not reseedNow and
      ((mocapNew and (not anchorExclusive or anchorPresent == SourceMocap))
        or (gpsNew and (gps.positionValid or gps.velocityValid)
          and (not anchorExclusive or anchorPresent == SourceGps)));
    opticalFlowAttempted := not reseedNow and opticalFlowNew
      and (not anchorExclusive or anchorPresent == SourceOpticalFlow);
    (working, mocapCorrectionAccepted, gpsPositionCorrectionAccepted,
     gpsVelocityCorrectionAccepted, correctionAttempted, correctionAccepted,
     correctionOutcome, correctionSource, normalizedInnovationSquared,
     mocapTimestampConsumedNext_s, gpsTimestampConsumedNext_s) :=
      correctAbsoluteAidingTick(working, imu, mocap, gps, tuning,
        gravityWorldEnu_m_s2, predictionAccepted, reseedNow,
        mocapNew, gpsNew, anchorExclusive, anchorPresent,
        imuTimestampHeldNext_s, imuAngularVelocityHeldNext_rad_s,
        imuSpecificForceHeldNext_m_s2, mocapTimestampConsumedPrevious_s,
        gpsTimestampConsumedPrevious_s);

    if opticalFlowAttempted then
      (working, opticalFlowCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctOpticalFlow(working, opticalFlow, tuning.innovationGate,
          imuTimestampHeldNext_s - opticalFlow.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s,
          tuning.minimumOpticalFlowQuality,
          tuning.minimumOpticalFlowGroundDistance_m,
          tuning.useSemiDirectBias);
      if not absoluteAidingAttempted then
        correctionAccepted := opticalFlowCorrectionAccepted;
        correctionSource := SourceOpticalFlow;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
      opticalFlowTimestampConsumedNext_s := opticalFlow.timestamp_s;
    end if;
    if not reseedNow and barometerNew then
      (working, barometerCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctBarometer(working, barometer,
          tuning.barometerBias_m, tuning.barometerBiasVariance_m2,
          tuning.innovationGate,
          imuTimestampHeldNext_s - barometer.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s,
          tuning.useSemiDirectBias, tuning.useBarometerBiasConsider,
          tuning.barometerBiasProcessNoise_m2_s);
      if not absoluteAidingAttempted and not opticalFlowAttempted then
        correctionAccepted := barometerCorrectionAccepted;
        correctionSource := SourceBarometer;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
      barometerTimestampConsumedNext_s := barometer.timestamp_s;
    end if;
    if not reseedNow and magnetometerNew then
      (working, magnetometerCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctMagnetometer(working, magnetometer,
          tuning.localMagneticFieldWorldEnu_T, tuning.innovationGate,
          imuTimestampHeldNext_s - magnetometer.timestamp_s,
          imuAngularVelocityHeldNext_rad_s,
          imuSpecificForceHeldNext_m_s2, gravityWorldEnu_m_s2,
          tuning.maximumAidingDelay_s, tuning.useEquivariantMagnetometer,
          tuning.useSemiDirectBias);
      if not absoluteAidingAttempted and not opticalFlowAttempted
          and not barometerNew then
        correctionAccepted := magnetometerCorrectionAccepted;
        correctionSource := SourceMagnetometer;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
      magnetometerTimestampConsumedNext_s := magnetometer.timestamp_s;
    end if;
    correctionAttempted := not reseedNow and (absoluteAidingAttempted
      or opticalFlowAttempted or barometerNew or magnetometerNew);
    pseudoPositionConfigured := tuning.pseudoPositionVariance_m2 > 0.0
      and tuning.pseudoPositionVariance_m2 < FiniteMagnitudeLimit;
    zeroVelocityConfigured := tuning.zeroVelocityVariance_m2_s2 > 0.0
      and tuning.zeroVelocityVariance_m2_s2 < FiniteMagnitudeLimit;
    if vehicleAtRest and imuUsable and not reseedNow
        and tuning.stationaryVelocityVariance_m2_s2 > 0.0
        and tuning.stationaryVelocityVariance_m2_s2 < FiniteMagnitudeLimit then
      (working, zeroVelocityCorrectionAccepted, aidingOutcome, aidingNis) :=
        correctZeroVelocity(working,
          tuning.stationaryVelocityVariance_m2_s2, 0.0,
          tuning.useSemiDirectBias);
      if not correctionAttempted then
        correctionAccepted := zeroVelocityCorrectionAccepted;
        correctionSource := SourceZeroVelocity;
        correctionOutcome := aidingOutcome;
        normalizedInnovationSquared := aidingNis;
      end if;
    end if;
    if not correctionAttempted and not zeroVelocityCorrectionAccepted
        and (not aidingLive or (ladderConfigured
          and rejectionElapsedPrevious_s
            >= tuning.aidingDivergentWindow_s)) then
      if zeroVelocityConfigured and imuQuiet then
        (working, zeroVelocityCorrectionAccepted, correctionOutcome,
         normalizedInnovationSquared) :=
          correctZeroVelocity(working, tuning.zeroVelocityVariance_m2_s2,
            0.0, tuning.useSemiDirectBias);
        correctionAccepted := zeroVelocityCorrectionAccepted;
        correctionSource := SourceZeroVelocity;
      elseif pseudoPositionConfigured then
        (working, pseudoPositionCorrectionAccepted, correctionOutcome,
         normalizedInnovationSquared) :=
          correctPseudoPosition(working, pseudoPositionHoldPrevious_m,
            tuning.pseudoPositionVariance_m2, 0.0,
            tuning.useSemiDirectBias);
        correctionAccepted := pseudoPositionCorrectionAccepted;
        correctionSource := SourcePseudoPosition;
      end if;
    end if;
    pseudoPositionHoldNext_m := if aidingLive
      then working.positionWorldEnu_m else pseudoPositionHoldPrevious_m;
  end if;
end advanceStateTick;
