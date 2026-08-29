within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2012 - 2025, PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function quatToRotationMatrix
  "Rotation matrix R (body to earth) from a Hamilton scalar-first quaternion"
  // Transcribes matrix::Dcm<Type>::Dcm(const Quaternion<Type>&),
  // src/lib/matrix/matrix/Dcm.hpp:85, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c. EKF2 uses R_to_earth = Dcm(quat_nominal).
  input Real q[4] "unit quaternion {w,x,y,z}";
  output Real R[3, 3] "rotation matrix, rotates a body-frame vector into earth (NED)";
protected
  Real a;
  Real b;
  Real c;
  Real d;
  Real ab;
  Real ac;
  Real ad;
  Real bb;
  Real bc;
  Real bd;
  Real cc;
  Real cd;
  Real dd;
algorithm
  a := q[1];
  b := q[2];
  c := q[3];
  d := q[4];
  ab := a * b;
  ac := a * c;
  ad := a * d;
  bb := b * b;
  bc := b * c;
  bd := b * d;
  cc := c * c;
  cd := c * d;
  dd := d * d;
  R[1, 1] := 1.0 - 2.0 * (cc + dd);
  R[1, 2] := 2.0 * (bc - ad);
  R[1, 3] := 2.0 * (ac + bd);
  R[2, 1] := 2.0 * (bc + ad);
  R[2, 2] := 1.0 - 2.0 * (bb + dd);
  R[2, 3] := 2.0 * (cd - ab);
  R[3, 1] := 2.0 * (bd - ac);
  R[3, 2] := 2.0 * (ab + cd);
  R[3, 3] := 1.0 - 2.0 * (bb + cc);
end quatToRotationMatrix;
