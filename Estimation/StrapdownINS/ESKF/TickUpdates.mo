within Estimation.StrapdownINS.ESKF;

package TickUpdates "Bounded gated state updates for the sampled dispatcher"
  function ReseedStateTick
    input State previous;
    input Boolean requested;
    input Integer anchorSource;
    input Avionics.MocapSample mocap;
    input Avionics.GpsSample gps;
    input Estimation.StrapdownINS.InitialVariances variances;
    output State next;
    output Boolean reseeded;
    output Integer source;
  protected
    Real position[3]; Real velocity[3];
  algorithm
    reseeded := requested;
    source := if requested then anchorSource else SourceNone;
    if requested then
      position := if anchorSource == SourceMocap then mocap.positionWorldEnu_m else gps.positionWorldEnu_m;
      velocity := if anchorSource == SourceGps and gps.velocityValid then gps.velocityWorldEnu_m_s else previous.velocityWorldEnu_m_s;
      next := reseed(previous, position, velocity, variances);
    else
      next := copyState(previous);
    end if;
  end ReseedStateTick;

  function OpticalFlowTick
    input State previous;
    input Tuning tuning;
    input Real gravityWorldEnu_m_s2[3];
    input Real imuTime;
    input Real angularVelocity[3];
    input Real specificForce[3];
    input Boolean requested;
    input Avionics.OpticalFlowSample opticalFlow;
    output State next;
    output Boolean accepted;
    output Integer outcome;
    output Real nis;
  algorithm
    accepted := false; outcome := CorrectionNotAttempted; nis := 0.0;
    if requested then
      (next, accepted, outcome, nis) := correctOpticalFlow(previous, opticalFlow,
        tuning.innovationGate, imuTime-opticalFlow.timestamp_s,
        angularVelocity, specificForce, gravityWorldEnu_m_s2,
        tuning.maximumAidingDelay_s, tuning.minimumOpticalFlowQuality,
        tuning.minimumOpticalFlowGroundDistance_m, tuning.useSemiDirectBias);
    else
      next := copyState(previous);
    end if;
  end OpticalFlowTick;

  function BarometerTick
    input State previous;
    input Tuning tuning;
    input Real gravityWorldEnu_m_s2[3];
    input Real imuTime;
    input Real angularVelocity[3];
    input Real specificForce[3];
    input Boolean requested;
    input Avionics.BarometerSample barometer;
    output State next;
    output Boolean accepted;
    output Integer outcome;
    output Real nis;
  algorithm
    accepted := false; outcome := CorrectionNotAttempted; nis := 0.0;
    if requested then
      (next, accepted, outcome, nis) := correctBarometer(previous, barometer,
        tuning.barometerBias_m, tuning.barometerBiasVariance_m2,
        tuning.innovationGate, imuTime-barometer.timestamp_s,
        angularVelocity, specificForce, gravityWorldEnu_m_s2,
        tuning.maximumAidingDelay_s, tuning.useSemiDirectBias,
        tuning.useBarometerBiasConsider, tuning.barometerBiasProcessNoise_m2_s);
    else
      next := copyState(previous);
    end if;
  end BarometerTick;

  function MagnetometerTick
    input State previous;
    input Tuning tuning;
    input Real gravityWorldEnu_m_s2[3];
    input Real imuTime;
    input Real angularVelocity[3];
    input Real specificForce[3];
    input Boolean requested;
    input Avionics.MagnetometerSample magnetometer;
    output State next;
    output Boolean accepted;
    output Integer outcome;
    output Real nis;
  algorithm
    accepted := false; outcome := CorrectionNotAttempted; nis := 0.0;
    if requested then
      (next, accepted, outcome, nis) := correctMagnetometer(previous, magnetometer,
        tuning.localMagneticFieldWorldEnu_T, tuning.innovationGate,
        imuTime-magnetometer.timestamp_s, angularVelocity, specificForce,
        gravityWorldEnu_m_s2, tuning.maximumAidingDelay_s,
        tuning.useEquivariantMagnetometer, tuning.useSemiDirectBias);
    else
      next := copyState(previous);
    end if;
  end MagnetometerTick;

  function StationaryVelocityTick
    input State previous;
    input Tuning tuning;
    input Boolean requested;
    output State next;
    output Boolean accepted;
    output Integer outcome;
    output Real nis;
  algorithm
    accepted := false; outcome := CorrectionNotAttempted; nis := 0.0;
    if requested then
      (next, accepted, outcome, nis) := correctZeroVelocity(previous,
        tuning.stationaryVelocityVariance_m2_s2, 0.0, tuning.useSemiDirectBias);
    else
      next := copyState(previous);
    end if;
  end StationaryVelocityTick;

  function FallbackHoldTick
    input State previous;
    input Tuning tuning;
    input Boolean requested;
    input Boolean imuQuiet;
    input Real positionHold[3];
    output State next;
    output Boolean attempted;
    output Boolean zeroVelocityAccepted;
    output Boolean pseudoPositionAccepted;
    output Integer outcome;
    output Integer source;
    output Real nis;
  protected
    Boolean zeroConfigured; Boolean positionConfigured;
  algorithm
    zeroConfigured := tuning.zeroVelocityVariance_m2_s2 > 0.0
      and tuning.zeroVelocityVariance_m2_s2 < FiniteMagnitudeLimit;
    positionConfigured := tuning.pseudoPositionVariance_m2 > 0.0
      and tuning.pseudoPositionVariance_m2 < FiniteMagnitudeLimit;
    attempted := requested and ((zeroConfigured and imuQuiet) or positionConfigured);
    zeroVelocityAccepted := false; pseudoPositionAccepted := false;
    outcome := CorrectionNotAttempted; source := SourceNone; nis := 0.0;
    if requested and zeroConfigured and imuQuiet then
      (next, zeroVelocityAccepted, outcome, nis) := correctZeroVelocity(previous,
        tuning.zeroVelocityVariance_m2_s2, 0.0, tuning.useSemiDirectBias);
      source := SourceZeroVelocity;
    elseif requested and positionConfigured then
      (next, pseudoPositionAccepted, outcome, nis) := correctPseudoPosition(previous,
        positionHold, tuning.pseudoPositionVariance_m2, 0.0, tuning.useSemiDirectBias);
      source := SourcePseudoPosition;
    else
      next := copyState(previous);
    end if;
  end FallbackHoldTick;
end TickUpdates;
