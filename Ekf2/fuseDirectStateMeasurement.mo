within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function fuseDirectStateMeasurement
  "Sequential scalar fusion of a direct state observation: Bierman-Joseph covariance update plus state correction"
  // Transcribes Ekf::fuseDirectStateMeasurement (ekf_helper.cpp:1034) and Ekf::fuse
  // (ekf_helper.cpp:740), PX4-Autopilot commit bd62df5e3ac3f3f4a07da4518062a922492adb6c.
  // The observation model is a unit row at stateIndex (H = e_k), so K = P[:,k]/S. The
  // covariance update is the efficient two-step Joseph-stabilized form; the state correction
  // uses EKF2's left-multiplicative attitude error and additive corrections for the other
  // states. clearInhibitedStateKalmanGains is a no-op under phase-1 flags (heading observable,
  // no bias inhibit). mag and terrain gains are zero on the GPS-position path (their
  // cross-covariance with position stays zero), so those corrections are structurally inert;
  // they are transcribed with EKF2's limits for completeness. Wind is inactive and untouched.
  input Real state[25];
  input Real P[24, 24];
  input Real innov "scalar innovation";
  input Real innovVar "innovation variance S";
  input Real R "observation variance";
  input Integer stateIndex "1-based tangent index of the observed state";
  input Real kGyroBiasVarMin;
  input Real kAccelBiasVarMin;
  input Real kMagVarMin;
  output Real stateOut[25];
  output Real Pout[24, 24];
protected
  Real K[24];
  Real Pw[24, 24];
  Real PH1[24];
  Real PH2[24];
  Real v;
  Real rot[3];
  Real dq[4];
  Real qNew[4];
algorithm
  // Kalman gain K = P[:,stateIndex] / S
  for row in 1:24 loop
    K[row] := P[row, stateIndex] / innovVar;
  end for;

  Pw := P;
  // Step 1 (conventional): PH = P.row(stateIndex); P(i,j) -= K(i)*PH(j)
  for j in 1:24 loop
    PH1[j] := Pw[stateIndex, j];
  end for;
  for i in 1:24 loop
    for j in 1:24 loop
      Pw[i, j] := Pw[i, j] - K[i] * PH1[j];
    end for;
  end for;
  // Step 2 (stabilized): PH = P.col(stateIndex); P(i,j) = P(i,j) - PH(i)*K(j) + K(i)*R*K(j), mirror
  for i in 1:24 loop
    PH2[i] := Pw[i, stateIndex];
  end for;
  for i in 1:24 loop
    for j in 1:24 loop
      if j <= i then
        v := Pw[i, j] - PH2[i] * K[j] + K[i] * R * K[j];
        Pw[i, j] := v;
        Pw[j, i] := v;
      end if;
    end for;
  end for;

  Pout := Ekf2.constrainStateVariances(Pw, kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin);

  // state correction (Ekf::fuse)
  stateOut := state;
  // quat_nominal: dq = axisAngle(K[quat] * -innov); q = dq * q (left multiply); normalize
  rot[1] := K[1] * (-1.0) * innov;
  rot[2] := K[2] * (-1.0) * innov;
  rot[3] := K[3] * (-1.0) * innov;
  dq := Ekf2.axisAngleToQuat(rot);
  qNew := Ekf2.quatNormalize(Ekf2.quatMultiply(dq, state[1:4]));
  stateOut[1:4] := qNew;
  // vel (constrain +-1e3)
  stateOut[5] := min(1000.0, max(-1000.0, state[5] - K[4] * innov));
  stateOut[6] := min(1000.0, max(-1000.0, state[6] - K[5] * innov));
  stateOut[7] := min(1000.0, max(-1000.0, state[7] - K[6] * innov));
  // pos: pos_correction = K * (-innov)
  stateOut[8] := state[8] + K[7] * (-1.0) * innov;
  stateOut[9] := state[9] + K[8] * (-1.0) * innov;
  stateOut[10] := state[10] + K[9] * (-1.0) * innov;
  // gyro bias (constrain +-0.4)
  stateOut[11] := min(0.4, max(-0.4, state[11] - K[10] * innov));
  stateOut[12] := min(0.4, max(-0.4, state[12] - K[11] * innov));
  stateOut[13] := min(0.4, max(-0.4, state[13] - K[12] * innov));
  // accel bias (constrain +-0.4)
  stateOut[14] := min(0.4, max(-0.4, state[14] - K[13] * innov));
  stateOut[15] := min(0.4, max(-0.4, state[15] - K[14] * innov));
  stateOut[16] := min(0.4, max(-0.4, state[16] - K[15] * innov));
  // mag_I (constrain +-1); gains structurally zero on this path
  stateOut[17] := min(1.0, max(-1.0, state[17] - K[16] * innov));
  stateOut[18] := min(1.0, max(-1.0, state[18] - K[17] * innov));
  stateOut[19] := min(1.0, max(-1.0, state[19] - K[18] * innov));
  // mag_B (constrain +-0.5)
  stateOut[20] := min(0.5, max(-0.5, state[20] - K[19] * innov));
  stateOut[21] := min(0.5, max(-0.5, state[21] - K[20] * innov));
  stateOut[22] := min(0.5, max(-0.5, state[22] - K[21] * innov));
  // wind (tangent 22,23): inactive, untouched -> stateOut[23], stateOut[24] keep state values
  // terrain (tangent 24 -> nominal 25): constrain +-1e4
  stateOut[25] := min(1e4, max(-1e4, state[25] - K[24] * innov));
end fuseDirectStateMeasurement;
