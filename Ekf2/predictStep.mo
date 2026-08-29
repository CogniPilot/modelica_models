within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function predictStep
  "One IMU prediction step: covariance prediction then nominal state prediction"
  // Composes Ekf2.predictCovarianceStep and Ekf2.predictState in EKF2's order
  // (predictCovariance then predictState, both on the pre-update state), matching
  // Ekf::update at src/modules/ekf2/EKF/ekf.cpp:182-183, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c.
  input Real state[25];
  input Real P[24, 24];
  input Real deltaAng[3];
  input Real deltaVel[3];
  input Real dtAng;
  input Real dtVel;
  input Real gyrNoise;
  input Real accNoise;
  input Real gyrBNoise;
  input Real accBNoise;
  input Real magENoise;
  input Real magBNoise;
  input Real magNoise;
  input Real kGyroBiasVarMin;
  input Real kAccelBiasVarMin;
  input Real kMagVarMin;
  input Real gravity;
  input Real earthRateNed[3];
  input Real rN;
  input Real rE;
  input Real alt0;
  input Real tanLat0;
  input Real velLim;
  output Real stateOut[25];
  output Real Pout[24, 24];
algorithm
  Pout := Ekf2.predictCovarianceStep(state, P, deltaAng, deltaVel, dtAng, dtVel,
    gyrNoise, accNoise, gyrBNoise, accBNoise, magENoise, magBNoise, magNoise,
    kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin);
  stateOut := Ekf2.predictState(state, deltaAng, deltaVel, dtAng, dtVel,
    gravity, earthRateNed, rN, rE, alt0, tanLat0, velLim);
end predictStep;
