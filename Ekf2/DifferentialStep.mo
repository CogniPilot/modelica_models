within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

block DifferentialStep
  "Fixed-sample EKF2 predict / GPS-position-fuse step block for the fidelity differential"
  // Holds EKF2's 25-vector nominal state and 24x24 error-state covariance across dosteps and
  // dispatches one operation per tick (load initial state, IMU prediction, or GPS horizontal
  // position fusion) selected by the Boolean inputs. This is the unit the benchmark compiles
  // with rumoca to embedded-c-galec and drives with identical inputs to the C++ oracle so the
  // per-step nominal state and full covariance can be compared. All math lives in the Ekf2
  // functions, each of which cites the PX4-Autopilot source it transcribes (commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c). Parameter defaults are the EKF2 defaults
  // (common.h) and the geodesy constants at the benchmark origin (40.4237 deg N, 180 m);
  // the driver overrides them to match the oracle bit for bit.

  parameter Real samplePeriod(unit = "s") = 0.02 "fixed sample period for the discrete block";

  // EKF2 covariance-prediction and process-noise parameters (common.h)
  parameter Real gyrNoise = 1.5e-2 "ekf2_gyr_noise";
  parameter Real accNoise = 3.5e-1 "ekf2_acc_noise";
  parameter Real gyrBNoise = 1.0e-3 "ekf2_gyr_b_noise";
  parameter Real accBNoise = 1.0e-2 "ekf2_acc_b_noise";
  parameter Real magENoise = 1.0e-3 "ekf2_mag_e_noise";
  parameter Real magBNoise = 1.0e-4 "ekf2_mag_b_noise";
  parameter Real magNoise = 5.0e-2 "ekf2_mag_noise";
  parameter Real kGyroBiasVarMin = 1e-9;
  parameter Real kAccelBiasVarMin = 1e-9;
  parameter Real kMagVarMin = 1e-6;

  // Geodesy constants at the EKF origin (evaluated once, held constant over the flight)
  parameter Real gravity = 9.8020747291557537 "WGS84 normal gravity at origin latitude";
  parameter Real earthRateNed[3] = {5.5512694952763948e-05, 0.0, -4.7284615007461736e-05};
  parameter Real rN = 6380241.8885338493 "meridian radius of curvature at origin";
  parameter Real rE = 6387132.4919517664 "transverse radius of curvature at origin";
  parameter Real alt0 = 180.0 "origin altitude";
  parameter Real tanLat0 = 0.85178021077334609 "tangent of origin latitude";
  parameter Real velLim = 100.0 "ekf2_vel_lim";

  // operation selectors (exactly one true per tick)
  input Boolean opLoad;
  input Boolean opPredict;
  input Boolean opFuse;

  // load inputs
  input Real inState[25];
  input Real inP[24, 24];
  // predict inputs
  input Real inDeltaAng[3];
  input Real inDeltaVel[3];
  input Real inDtAng;
  input Real inDtVel;
  // fuse inputs
  input Real inObsN;
  input Real inObsE;
  input Real inPosVar;
  input Real inGate;

  // outputs
  output Real outState[25];
  output Real outP[24, 24];
  output Boolean outFused;

protected
  discrete Real stateVec[25](each start = 0.0, each fixed = true);
  discrete Real covP[24, 24](each start = 0.0, each fixed = true);
algorithm
  when sample(0.0, samplePeriod) then
    if opLoad then
      stateVec := inState;
      covP := inP;
      outFused := false;
    elseif opPredict then
      (stateVec, covP) := Ekf2.predictStep(pre(stateVec), pre(covP),
        inDeltaAng, inDeltaVel, inDtAng, inDtVel,
        gyrNoise, accNoise, gyrBNoise, accBNoise, magENoise, magBNoise, magNoise,
        kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin,
        gravity, earthRateNed, rN, rE, alt0, tanLat0, velLim);
      outFused := false;
    elseif opFuse then
      (stateVec, covP, outFused) := Ekf2.gpsHorizontalPositionFuse(pre(stateVec), pre(covP),
        inObsN, inObsE, inPosVar, inGate,
        kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin);
    else
      stateVec := pre(stateVec);
      covP := pre(covP);
      outFused := false;
    end if;
    outState := stateVec;
    outP := covP;
  end when;
  annotation(
    Documentation(info = "<html><p>See <code>Ekf2</code> package documentation for provenance,
license (PX4-Autopilot, BSD-3-Clause, commit bd62df5e3ac3f3f4a07da4518062a922492adb6c), and
references. This block is a benchmark fixture, not the estimator interface; it exposes EKF2's
raw predict and GPS-position-fuse operations for a per-step fidelity comparison.</p></html>"));
end DifferentialStep;
