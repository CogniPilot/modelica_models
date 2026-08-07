within Estimation;
model PositionEstimator
  "Fixed-gain GPS/mocap position estimator with inertial propagation"
  parameter Real samplePeriod(unit="s") = 0.005;
  parameter Real gravity_m_s2(unit="m/s2") = 9.81;
  parameter Real initialPosition_m[3] = {0.0, 0.0, 0.0};
  parameter Real initialVelocity_m_s[3] = {0.0, 0.0, 0.0};
  parameter Real steadyStateGain[6, 3] = [
    0.05, 0.0, 0.0;
    0.0, 0.05, 0.0;
    0.0, 0.0, 0.05;
    0.03, 0.0, 0.0;
    0.0, 0.03, 0.0;
    0.0, 0.0, 0.03]
    "Fixed gain; upper rows correct position and lower rows correct velocity";

  input Real attitude_wb[4](start={1.0, 0.0, 0.0, 0.0})
    "Estimated scalar-first quaternion from body FLU to world ENU";
  input Real specificForce_b_m_s2[3](start={0.0, 0.0, gravity_m_s2})
    "Raw accelerometer specific force in body FLU [m/s2]";
  input Real measuredPosition_m[3](each start=0.0)
    "Local Cartesian GPS or mocap position in world ENU [m]";
  input Boolean measurementFresh(start=false)
    "True for exactly one estimator tick when a new valid measurement arrives";
  input Boolean reset(start=true);

  discrete output Real position_m[3](each start=0.0)
    "Estimated position in world ENU [m]";
  discrete output Real velocity_m_s[3](each start=0.0)
    "Estimated velocity in world ENU [m/s]";
  discrete output Real acceleration_m_s2[3](each start=0.0)
    "Estimated inertial acceleration in world ENU [m/s2]";
  discrete output Real positionResidual_m[3](each start=0.0)
    "Latest accepted position innovation [m]";
  discrete output Boolean correctionApplied(start=false);

protected
  discrete Real predictedPosition_m[3](each start=0.0);
  discrete Real predictedVelocity_m_s[3](each start=0.0);
  discrete Real stateCorrection[6](each start=0.0);
  discrete Real attitudeNorm(start=1.0);
  discrete Real normalizedAttitude[4](start={1.0, 0.0, 0.0, 0.0});

algorithm
  when sample(0.0, samplePeriod) then
    attitudeNorm := max(sqrt(
      attitude_wb[1] * attitude_wb[1]
        + attitude_wb[2] * attitude_wb[2]
        + attitude_wb[3] * attitude_wb[3]
        + attitude_wb[4] * attitude_wb[4]), 1.0e-10);
    normalizedAttitude := attitude_wb / attitudeNorm;
    acceleration_m_s2[1] :=
      (1.0 - 2.0 * (normalizedAttitude[3] * normalizedAttitude[3]
        + normalizedAttitude[4] * normalizedAttitude[4]))
        * specificForce_b_m_s2[1]
      + 2.0 * (normalizedAttitude[2] * normalizedAttitude[3]
        - normalizedAttitude[1] * normalizedAttitude[4])
        * specificForce_b_m_s2[2]
      + 2.0 * (normalizedAttitude[2] * normalizedAttitude[4]
        + normalizedAttitude[1] * normalizedAttitude[3])
        * specificForce_b_m_s2[3];
    acceleration_m_s2[2] :=
      2.0 * (normalizedAttitude[2] * normalizedAttitude[3]
        + normalizedAttitude[1] * normalizedAttitude[4])
        * specificForce_b_m_s2[1]
      + (1.0 - 2.0 * (normalizedAttitude[2] * normalizedAttitude[2]
        + normalizedAttitude[4] * normalizedAttitude[4]))
        * specificForce_b_m_s2[2]
      + 2.0 * (normalizedAttitude[3] * normalizedAttitude[4]
        - normalizedAttitude[1] * normalizedAttitude[2])
        * specificForce_b_m_s2[3];
    acceleration_m_s2[3] :=
      2.0 * (normalizedAttitude[2] * normalizedAttitude[4]
        - normalizedAttitude[1] * normalizedAttitude[3])
        * specificForce_b_m_s2[1]
      + 2.0 * (normalizedAttitude[3] * normalizedAttitude[4]
        + normalizedAttitude[1] * normalizedAttitude[2])
        * specificForce_b_m_s2[2]
      + (1.0 - 2.0 * (normalizedAttitude[2] * normalizedAttitude[2]
        + normalizedAttitude[3] * normalizedAttitude[3]))
        * specificForce_b_m_s2[3]
      - gravity_m_s2;
    predictedPosition_m := if reset then
      (if measurementFresh then measuredPosition_m else initialPosition_m)
      else pre(position_m) + pre(velocity_m_s) * samplePeriod
        + 0.5 * acceleration_m_s2 * samplePeriod * samplePeriod;
    predictedVelocity_m_s := if reset then initialVelocity_m_s
      else pre(velocity_m_s) + acceleration_m_s2 * samplePeriod;
    positionResidual_m := if not reset and measurementFresh then
      measuredPosition_m - predictedPosition_m else {0.0, 0.0, 0.0};
    stateCorrection[1] := steadyStateGain[1, 1] * positionResidual_m[1]
      + steadyStateGain[1, 2] * positionResidual_m[2]
      + steadyStateGain[1, 3] * positionResidual_m[3];
    stateCorrection[2] := steadyStateGain[2, 1] * positionResidual_m[1]
      + steadyStateGain[2, 2] * positionResidual_m[2]
      + steadyStateGain[2, 3] * positionResidual_m[3];
    stateCorrection[3] := steadyStateGain[3, 1] * positionResidual_m[1]
      + steadyStateGain[3, 2] * positionResidual_m[2]
      + steadyStateGain[3, 3] * positionResidual_m[3];
    stateCorrection[4] := steadyStateGain[4, 1] * positionResidual_m[1]
      + steadyStateGain[4, 2] * positionResidual_m[2]
      + steadyStateGain[4, 3] * positionResidual_m[3];
    stateCorrection[5] := steadyStateGain[5, 1] * positionResidual_m[1]
      + steadyStateGain[5, 2] * positionResidual_m[2]
      + steadyStateGain[5, 3] * positionResidual_m[3];
    stateCorrection[6] := steadyStateGain[6, 1] * positionResidual_m[1]
      + steadyStateGain[6, 2] * positionResidual_m[2]
      + steadyStateGain[6, 3] * positionResidual_m[3];
    position_m[1] := predictedPosition_m[1] + stateCorrection[1];
    position_m[2] := predictedPosition_m[2] + stateCorrection[2];
    position_m[3] := predictedPosition_m[3] + stateCorrection[3];
    velocity_m_s[1] := predictedVelocity_m_s[1] + stateCorrection[4];
    velocity_m_s[2] := predictedVelocity_m_s[2] + stateCorrection[5];
    velocity_m_s[3] := predictedVelocity_m_s[3] + stateCorrection[6];
    correctionApplied := measurementFresh;
  end when;
end PositionEstimator;
