within Ekf2;

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
// Transcribed from PX4-Autopilot at commit
// bd62df5e3ac3f3f4a07da4518062a922492adb6c. The upstream copyright notice,
// the three BSD-3-Clause conditions, and the warranty disclaimer are retained
// verbatim in Ekf2/LICENSE, as condition 1 of that license requires.

function predictCovarianceStep
  "Full covariance prediction: SymForce propagation, stationary-state process noise, symmetrize, constrain"
  // Transcribes Ekf::predictCovariance, src/modules/ekf2/EKF/covariance.cpp:113, PX4-Autopilot
  // commit bd62df5e3ac3f3f4a07da4518062a922492adb6c. Calls Ekf2.PredictCovariance (the SymForce
  // Jacobian propagation), then adds the diagonal process-noise growth for the stationary-model
  // states, copies the upper triangle to the lower (symmetry), and applies the variance limiter.
  // Phase-1 scope: heading is observable so no heading uncorrelation; wind and terrain process
  // noise are frozen (states inert on the IMU + GPS-position path); accel is never flagged bad.
  input Real state[25];
  input Real P[24, 24];
  input Real deltaAng[3];
  input Real deltaVel[3];
  input Real dtAng;
  input Real dtVel;
  input Real gyrNoise "ekf2_gyr_noise";
  input Real accNoise "ekf2_acc_noise";
  input Real gyrBNoise "ekf2_gyr_b_noise";
  input Real accBNoise "ekf2_acc_b_noise";
  input Real magENoise "ekf2_mag_e_noise";
  input Real magBNoise "ekf2_mag_b_noise";
  input Real magNoise "ekf2_mag_noise";
  input Real kGyroBiasVarMin;
  input Real kAccelBiasVarMin;
  input Real kMagVarMin;
  output Real Pout[24, 24];
protected
  Real dt;
  Real gyroVar;
  Real accelVar;
  Real accel[3];
  Real gyro[3];
  Real Pw[24, 24];
  Real sg;
  Real png;
  Real sa;
  Real pna;
  Real thr;
  Real smi;
  Real pnmi;
  Real smb;
  Real pnmb;
algorithm
  dt := 0.5 * (dtVel + dtAng);
  gyroVar := gyrNoise * gyrNoise;
  accelVar := accNoise * accNoise;
  accel := deltaVel / dtVel;
  gyro := deltaAng / dtAng;

  Pw := Ekf2.PredictCovariance(state, P, accel, {accelVar, accelVar, accelVar}, gyro, gyroVar, dt);

  // gyro bias process noise (tangent 10:12)
  sg := dt * gyrBNoise;
  png := sg * sg;
  if Pw[10, 10] < gyroVar then Pw[10, 10] := Pw[10, 10] + png; end if;
  if Pw[11, 11] < gyroVar then Pw[11, 11] := Pw[11, 11] + png; end if;
  if Pw[12, 12] < gyroVar then Pw[12, 12] := Pw[12, 12] + png; end if;
  // accel bias process noise (tangent 13:15)
  sa := dt * accBNoise;
  pna := sa * sa;
  if Pw[13, 13] < accelVar then Pw[13, 13] := Pw[13, 13] + pna; end if;
  if Pw[14, 14] < accelVar then Pw[14, 14] := Pw[14, 14] + pna; end if;
  if Pw[15, 15] < accelVar then Pw[15, 15] := Pw[15, 15] + pna; end if;
  // mag_I process noise (tangent 16:18)
  thr := magNoise * magNoise;
  smi := dt * magENoise;
  pnmi := smi * smi;
  if Pw[16, 16] < thr then Pw[16, 16] := Pw[16, 16] + pnmi; end if;
  if Pw[17, 17] < thr then Pw[17, 17] := Pw[17, 17] + pnmi; end if;
  if Pw[18, 18] < thr then Pw[18, 18] := Pw[18, 18] + pnmi; end if;
  // mag_B process noise (tangent 19:21)
  smb := dt * magBNoise;
  pnmb := smb * smb;
  if Pw[19, 19] < thr then Pw[19, 19] := Pw[19, 19] + pnmb; end if;
  if Pw[20, 20] < thr then Pw[20, 20] := Pw[20, 20] + pnmb; end if;
  if Pw[21, 21] < thr then Pw[21, 21] := Pw[21, 21] + pnmb; end if;
  // wind, terrain process noise: frozen in phase-1

  // symmetrize: copy upper triangle to lower (covariance.cpp:243)
  for r in 1:24 loop
    for c in 1:24 loop
      if c < r then
        Pw[r, c] := Pw[c, r];
      end if;
    end for;
  end for;

  Pout := Ekf2.constrainStateVariances(Pw, kGyroBiasVarMin, kAccelBiasVarMin, kMagVarMin);
end predictCovarianceStep;
