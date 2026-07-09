within Estimation.MocapExternalOdometryIEKF;
function correct "Discrete pose correction for the mocap IEKF"
  input Real x_pred[157] "Predicted flat state";
  input Real measurement_position_enu_m[3] "Measured position [m]";
  input Real measurement_attitude_wxyz[4] "Measured attitude quaternion";
  input Real attitudeMeasurementVariance "Attitude measurement variance [rad2]";
  input Real positionMeasurementVariance "Position measurement variance [m2]";
  output Real x_next[157] "Corrected flat state";
  output Real accepted "1 when correction succeeded, otherwise 0";
protected
  Integer observed[6] = {1, 2, 3, 7, 8, 9};
  Real state_pred[13];
  Real state_next[13];
  Real covariance_pred[12, 12];
  Real covariance_next[12, 12];
  Real residual[6];
  Real attitude_residual[3];
  Real dq[4];
  Real innovation_covariance[6, 6];
  Real innovation_covariance_inv[6, 6];
  Real inverse_ok;
  Real gain[12, 6];
  Real correction[12];
  Real a[12, 12];
  Real measurement_noise[6, 6];
algorithm
  state_pred := x_pred[1:13];
  for row in 1:12 loop
    for col in 1:12 loop
      covariance_pred[row, col] := x_pred[13 + (col - 1) * 12 + row];
    end for;
  end for;

  dq := LieGroups.SO3.Quat.product(
    LieGroups.SO3.Quat.inverse(state_pred[1:4]),
    LieGroups.SO3.Quat.normalize(measurement_attitude_wxyz));
  attitude_residual := LieGroups.SO3.Quat.log_map(dq);
  for i in 1:3 loop
    residual[i] := attitude_residual[i];
    residual[i + 3] := measurement_position_enu_m[i] - state_pred[i + 7];
  end for;

  for row in 1:6 loop
    for col in 1:6 loop
      innovation_covariance[row, col] :=
        covariance_pred[observed[row], observed[col]];
      measurement_noise[row, col] := 0.0;
    end for;
  end for;
  for i in 1:3 loop
    innovation_covariance[i, i] :=
      innovation_covariance[i, i] + attitudeMeasurementVariance;
    innovation_covariance[i + 3, i + 3] :=
      innovation_covariance[i + 3, i + 3] + positionMeasurementVariance;
    measurement_noise[i, i] := attitudeMeasurementVariance;
    measurement_noise[i + 3, i + 3] := positionMeasurementVariance;
  end for;

  (innovation_covariance_inv, inverse_ok) :=
    Estimation.MocapExternalOdometryIEKF.inverse6(innovation_covariance);
  if inverse_ok < 0.5 then
    x_next := x_pred;
    accepted := 0.0;
  else
    for row in 1:12 loop
      for meas in 1:6 loop
        gain[row, meas] := 0.0;
        for k in 1:6 loop
          gain[row, meas] := gain[row, meas]
            + covariance_pred[row, observed[k]]
            * innovation_covariance_inv[k, meas];
        end for;
      end for;
    end for;

    for row in 1:12 loop
      correction[row] := 0.0;
      for meas in 1:6 loop
        correction[row] := correction[row] + gain[row, meas] * residual[meas];
      end for;
    end for;

    state_next :=
      Estimation.MocapExternalOdometryIEKF.inject(state_pred, correction);

    a := identity(12);
    for row in 1:12 loop
      a[row, 1] := a[row, 1] - gain[row, 1];
      a[row, 2] := a[row, 2] - gain[row, 2];
      a[row, 3] := a[row, 3] - gain[row, 3];
      a[row, 7] := a[row, 7] - gain[row, 4];
      a[row, 8] := a[row, 8] - gain[row, 5];
      a[row, 9] := a[row, 9] - gain[row, 6];
    end for;
    covariance_next := Estimation.MocapExternalOdometryIEKF.joseph_update(
      a,
      covariance_pred,
      gain,
      measurement_noise);
    covariance_next :=
      Estimation.MocapExternalOdometryIEKF.stabilize_covariance(covariance_next);

    for i in 1:13 loop
      x_next[i] := state_next[i];
    end for;
    for row in 1:12 loop
      for col in 1:12 loop
        x_next[13 + (col - 1) * 12 + row] := covariance_next[row, col];
      end for;
    end for;
    accepted := 1.0;
  end if;
end correct;
