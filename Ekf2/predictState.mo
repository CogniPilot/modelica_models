within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function predictState
  "Nominal strapdown state prediction (quaternion, velocity, position) for one IMU packet"
  // Transcribes Ekf::predictState, src/modules/ekf2/EKF/ekf.cpp:230, PX4-Autopilot commit
  // bd62df5e3ac3f3f4a07da4518062a922492adb6c. Bias-corrected strapdown integration with
  // earth-rotation removal on the delta angle, WGS84 gravity, Coriolis, and transport-rate
  // terms. Phase-1 scope: earth rate, gravity, and radii of curvature are held constant at
  // the origin latitude (EKF2 recomputes them only when latitude moves > 1 deg, ekf.cpp:231);
  // position is integrated flat in local NED (EKF2's curvilinear LatLonAlt bookkeeping is a
  // sub-1e-9 m effect over the benchmark's local flight and the covariance model itself
  // integrates position flat, derivation.py). Only quat/vel/pos change; other states pass through.
  input Real stateIn[25];
  input Real deltaAng[3] "delta angle over the packet (FRD, rad)";
  input Real deltaVel[3] "delta velocity over the packet (FRD, m/s)";
  input Real dtAng "delta-angle integration time (s)";
  input Real dtVel "delta-velocity integration time (s)";
  input Real gravity "WGS84 normal gravity at origin latitude (m/s2, positive down)";
  input Real earthRateNed[3] "earth rotation rate in NED at origin latitude (rad/s)";
  input Real rN "meridian radius of curvature at origin (m)";
  input Real rE "transverse radius of curvature at origin (m)";
  input Real alt0 "origin altitude (m)";
  input Real tanLat0 "tangent of origin latitude";
  input Real velLim "velocity state limit (m/s), ekf2_vel_lim";
  output Real stateOut[25];
protected
  Real qOld[4];
  Real Rold[3, 3];
  Real Rnew[3, 3];
  Real gyroBias[3];
  Real accelBias[3];
  Real correctedDeltaAng[3];
  Real dq[4];
  Real qNew[4];
  Real correctedDeltaVel[3];
  Real dvelEf[3];
  Real velLast[3];
  Real vel[3];
  Real coriolis[3];
  Real angRateNav[3];
  Real transport[3];
  Real posPrev[3];
  Real pos[3];
algorithm
  stateOut := stateIn;
  qOld := stateIn[1:4];
  gyroBias := stateIn[11:13];
  accelBias := stateIn[14:16];
  velLast := stateIn[5:7];
  posPrev := stateIn[8:10];

  Rold := Ekf2.quatToRotationMatrix(qOld);
  // corrected_delta_ang = delta_ang - gyro_bias*dt - R^T * earth_rate_NED * dt
  correctedDeltaAng := deltaAng - gyroBias * dtAng - (transpose(Rold) * earthRateNed) * dtAng;
  dq := Ekf2.axisAngleToQuat(correctedDeltaAng);
  qNew := Ekf2.quatNormalize(Ekf2.quatMultiply(qOld, dq));
  Rnew := Ekf2.quatToRotationMatrix(qNew);

  correctedDeltaVel := deltaVel - accelBias * dtVel;
  dvelEf := Rnew * correctedDeltaVel;
  vel := velLast + dvelEf;

  // Coriolis: -2 * earth_rate_NED x vel_last
  coriolis[1] := (-2.0) * (earthRateNed[2] * velLast[3] - earthRateNed[3] * velLast[2]);
  coriolis[2] := (-2.0) * (earthRateNed[3] * velLast[1] - earthRateNed[1] * velLast[3]);
  coriolis[3] := (-2.0) * (earthRateNed[1] * velLast[2] - earthRateNed[2] * velLast[1]);

  // transport rate: -computeAngularRateNavFrame(vel_last) x vel_last (lat_lon_alt.cpp)
  angRateNav[1] := velLast[2] / (rE + alt0);
  angRateNav[2] := (-velLast[1]) / (rN + alt0);
  angRateNav[3] := (-velLast[2]) * tanLat0 / (rE + alt0);
  transport[1] := (-1.0) * (angRateNav[2] * velLast[3] - angRateNav[3] * velLast[2]);
  transport[2] := (-1.0) * (angRateNav[3] * velLast[1] - angRateNav[1] * velLast[3]);
  transport[3] := (-1.0) * (angRateNav[1] * velLast[2] - angRateNav[2] * velLast[1]);

  vel := vel + ({0.0, 0.0, gravity} + coriolis + transport) * dtVel;
  pos := posPrev + (velLast + vel) * dtVel * 0.5;

  vel[1] := min(velLim, max(-velLim, vel[1]));
  vel[2] := min(velLim, max(-velLim, vel[2]));
  vel[3] := min(velLim, max(-velLim, vel[3]));

  stateOut[1:4] := qNew;
  stateOut[5:7] := vel;
  stateOut[8:10] := pos;
end predictState;
