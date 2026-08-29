within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2024 PX4 Development Team. All rights reserved.
// Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function gpsHorizontalPositionFuse
  "GPS horizontal-position fusion: 2D innovation gate then sequential N,E direct-state fusion"
  // Transcribes the observation and gate from Ekf::updateGnssPos
  // (aid_sources/gnss/gps_control.cpp:344) and the templated 2D updateAidSourceStatus
  // (ekf.h:1117), and the sequential fusion from Ekf::fuseHorizontalPosition
  // (position_fusion.cpp:56), PX4-Autopilot commit bd62df5e3ac3f3f4a07da4518062a922492adb6c.
  // The observation (obsN, obsE) is the GPS position resolved into local NED, and posVar / gate
  // are precomputed by the caller (max(hacc,ekf2_gps_p_noise) with the no-aiding clamp, squared
  // and floored; gate = max(ekf2_gps_p_gate,1)). The 2D gate rejects the whole update if either
  // axis test ratio exceeds one; otherwise N then E are fused sequentially, the E innovation and
  // both innovation variances recomputed from the state and covariance updated by the N fusion.
  input Real state[25];
  input Real P[24, 24];
  input Real obsN "GPS north position, local NED (m)";
  input Real obsE "GPS east position, local NED (m)";
  input Real posVar "observation variance (m2)";
  input Real gate "innovation gate size (STD)";
  input Real kGyroBiasVarMin;
  input Real kAccelBiasVarMin;
  input Real kMagVarMin;
  output Real stateOut[25];
  output Real Pout[24, 24];
  output Boolean fused;
protected
  Real innov0;
  Real innov1;
  Real ivar0;
  Real ivar1;
  Real tr0;
  Real tr1;
  Real posPrev0;
  Real posPrev1;
  Real innovN;
  Real ivarN;
  Real innovE;
  Real ivarE;
  Real s1[25];
  Real P1[24, 24];
algorithm
  // innovation = predicted - measured; innovation variance = P_pos + obs_var (nominal pos N,E = state[8],state[9]; tangent = P[7,7],P[8,8])
  innov0 := state[8] - obsN;
  innov1 := state[9] - obsE;
  ivar0 := P[7, 7] + posVar;
  ivar1 := P[8, 8] + posVar;
  tr0 := (innov0 * innov0) / (gate * gate * ivar0);
  tr1 := (innov1 * innov1) / (gate * gate * ivar1);

  if (tr0 > 1.0) or (tr1 > 1.0) then
    stateOut := state;
    Pout := P;
    fused := false;
  else
    posPrev0 := state[8];
    posPrev1 := state[9];
    // fuse N (tangent index 7)
    innovN := innov0 + (state[8] - posPrev0);
    ivarN := P[7, 7] + posVar;
    (s1, P1) := Ekf2.fuseDirectStateMeasurement(state, P, innovN, ivarN, posVar, 7,
      kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin);
    // fuse E (tangent index 8), innovation corrected by the E position change from the N fusion
    innovE := innov1 + (s1[9] - posPrev1);
    ivarE := P1[8, 8] + posVar;
    (stateOut, Pout) := Ekf2.fuseDirectStateMeasurement(s1, P1, innovE, ivarE, posVar, 8,
      kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin);
    fused := true;
  end if;
end gpsHorizontalPositionFuse;
