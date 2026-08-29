within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2012 - 2025, PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function quatNormalize
  "Normalize a quaternion to unit norm"
  // Transcribes matrix::Vector<Type,4>::normalize() (divide by sqrt of the sum of
  // squares), src/lib/matrix/matrix/Vector.hpp:122, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c.
  input Real q[4];
  output Real r[4];
protected
  Real n;
algorithm
  n := sqrt(q[1] * q[1] + q[2] * q[2] + q[3] * q[3] + q[4] * q[4]);
  r[1] := q[1] / n;
  r[2] := q[2] / n;
  r[3] := q[3] / n;
  r[4] := q[4] / n;
end quatNormalize;
