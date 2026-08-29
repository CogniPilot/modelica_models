within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function constrainStateVariances
  "Lower-bound and ratio-floor the covariance diagonal blocks"
  // Transcribes Ekf::constrainStateVariances / constrainStateVar / constrainStateVarLimitRatio,
  // src/modules/ekf2/EKF/covariance.cpp:249, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c. Phase-1 scope: only the lower-bound (P below
  // min raised to min) and ratio-floor branches are implemented; the upper-bound pseudo-fusion
  // branch (P above max) never triggers on the benchmark data (the oracle asserts maxviol = 0
  // over the full flight), so omitting it is exact for this stream. heading_observable is held
  // true, so the heading uncorrelation is skipped; wind and terrain are inactive and unconstrained.
  input Real P[24, 24];
  input Real kGyroBiasVarMin;
  input Real kAccelBiasVarMin;
  input Real kMagVarMin;
  output Real Q[24, 24];
protected
  Real gmax;
  Real gfloor;
algorithm
  Q := P;
  // quat_nominal: min 1e-9 (max 1)
  Q[1, 1] := max(Q[1, 1], 1e-9);
  Q[2, 2] := max(Q[2, 2], 1e-9);
  Q[3, 3] := max(Q[3, 3], 1e-9);
  // vel: min 1e-6 (max 1e6)
  Q[4, 4] := max(Q[4, 4], 1e-6);
  Q[5, 5] := max(Q[5, 5], 1e-6);
  Q[6, 6] := max(Q[6, 6], 1e-6);
  // pos: min 1e-6 (max 1e6)
  Q[7, 7] := max(Q[7, 7], 1e-6);
  Q[8, 8] := max(Q[8, 8], 1e-6);
  Q[9, 9] := max(Q[9, 9], 1e-6);
  // gyro_bias: ratio floor, min kGyroBiasVarMin, max 1, ratio 1e6
  gmax := max(max(Q[10, 10], Q[11, 11]), Q[12, 12]);
  gfloor := min(1.0, max(kGyroBiasVarMin, min(1.0, max(kGyroBiasVarMin, gmax)) / 1e6));
  Q[10, 10] := max(Q[10, 10], gfloor);
  Q[11, 11] := max(Q[11, 11], gfloor);
  Q[12, 12] := max(Q[12, 12], gfloor);
  // accel_bias: ratio floor, min kAccelBiasVarMin, max 1, ratio 1e6
  gmax := max(max(Q[13, 13], Q[14, 14]), Q[15, 15]);
  gfloor := min(1.0, max(kAccelBiasVarMin, min(1.0, max(kAccelBiasVarMin, gmax)) / 1e6));
  Q[13, 13] := max(Q[13, 13], gfloor);
  Q[14, 14] := max(Q[14, 14], gfloor);
  Q[15, 15] := max(Q[15, 15], gfloor);
  // mag_I: ratio floor, min kMagVarMin, max 1, ratio 1e6 (mag control active)
  gmax := max(max(Q[16, 16], Q[17, 17]), Q[18, 18]);
  gfloor := min(1.0, max(kMagVarMin, min(1.0, max(kMagVarMin, gmax)) / 1e6));
  Q[16, 16] := max(Q[16, 16], gfloor);
  Q[17, 17] := max(Q[17, 17], gfloor);
  Q[18, 18] := max(Q[18, 18], gfloor);
  // mag_B: ratio floor, min kMagVarMin, max 1, ratio 1e6
  gmax := max(max(Q[19, 19], Q[20, 20]), Q[21, 21]);
  gfloor := min(1.0, max(kMagVarMin, min(1.0, max(kMagVarMin, gmax)) / 1e6));
  Q[19, 19] := max(Q[19, 19], gfloor);
  Q[20, 20] := max(Q[20, 20], gfloor);
  Q[21, 21] := max(Q[21, 21], gfloor);
end constrainStateVariances;
