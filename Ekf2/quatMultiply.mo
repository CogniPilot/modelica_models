within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2012 - 2025, PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function quatMultiply
  "Hamilton quaternion product q * p, scalar-first {w,x,y,z}"
  // Transcribes matrix::Quaternion<Type>::operator*(const Quaternion&),
  // src/lib/matrix/matrix/Quaternion.hpp, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c.
  input Real q[4];
  input Real p[4];
  output Real r[4];
algorithm
  r[1] := q[1] * p[1] - q[2] * p[2] - q[3] * p[3] - q[4] * p[4];
  r[2] := q[2] * p[1] + q[1] * p[2] - q[4] * p[3] + q[3] * p[4];
  r[3] := q[3] * p[1] + q[4] * p[2] + q[1] * p[3] - q[2] * p[4];
  r[4] := q[4] * p[1] - q[3] * p[2] + q[2] * p[3] + q[1] * p[4];
end quatMultiply;
