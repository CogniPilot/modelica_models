within Estimation.MocapExternalOdometryIEKF;
function predict "Discrete SE_2(3) prediction for one mocap IEKF step"
  input Real x_prev[157]
    "Flat state: attitude, velocity, position, angular velocity, covariance";
  input Real dt_s "Prediction interval [s]";
  input Real process_variance[4]
    "{attitude, velocity, position, angular velocity} spectral densities";
  output Real x_pred[157] "Predicted flat state";
protected
  Real state_prev[13];
  Real state_pred[13];
  Real covariance_prev[12, 12];
  Real covariance_pred[12, 12];
  Real transition[12, 12];
  Real transition_covariance[12, 12];
  Real se23_state[10];
  Real se23_prediction[10];
  Real attitude_pred[4];
  Real left_increment[9];
  Real zero_increment[9];
  Real coupling[2, 2];
algorithm
  state_prev := x_prev[1:13];
  for row in 1:12 loop
    for col in 1:12 loop
      covariance_prev[row, col] := x_prev[13 + (col - 1) * 12 + row];
    end for;
  end for;

  se23_state := {
    state_prev[8], state_prev[9], state_prev[10],
    state_prev[5], state_prev[6], state_prev[7],
    state_prev[1], state_prev[2], state_prev[3], state_prev[4]};
  left_increment := {
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    state_prev[11] * dt_s,
    state_prev[12] * dt_s,
    state_prev[13] * dt_s};
  zero_increment := zeros(9);
  coupling := [0.0, dt_s; 0.0, 0.0];

  se23_prediction := LieGroups.SE23.Quat.exp_mixed(
    se23_state,
    left_increment,
    zero_increment,
    coupling);

  attitude_pred :=
    LieGroups.SO3.Quat.normalize(se23_prediction[7:10]);
  for i in 1:4 loop
    state_pred[i] := attitude_pred[i];
  end for;
  for i in 1:3 loop
    state_pred[i + 4] := se23_prediction[i + 3];
    state_pred[i + 7] := se23_prediction[i];
    state_pred[i + 10] := state_prev[i + 10];
  end for;

  transition := identity(12);
  for axis in 1:3 loop
    transition[axis, axis + 9] := dt_s;
    transition[axis + 6, axis + 3] := dt_s;
  end for;
  transition_covariance := transition * covariance_prev;
  covariance_pred := transition_covariance * transpose(transition);
  for axis in 1:3 loop
    covariance_pred[axis, axis] :=
      covariance_pred[axis, axis] + process_variance[1] * dt_s;
    covariance_pred[axis + 3, axis + 3] :=
      covariance_pred[axis + 3, axis + 3] + process_variance[2] * dt_s;
    covariance_pred[axis + 6, axis + 6] :=
      covariance_pred[axis + 6, axis + 6] + process_variance[3] * dt_s;
    covariance_pred[axis + 9, axis + 9] :=
      covariance_pred[axis + 9, axis + 9] + process_variance[4] * dt_s;
  end for;
  covariance_pred :=
    Estimation.MocapExternalOdometryIEKF.stabilize_covariance(covariance_pred);

  for i in 1:13 loop
    x_pred[i] := state_pred[i];
  end for;
  for row in 1:12 loop
    for col in 1:12 loop
      x_pred[13 + (col - 1) * 12 + row] := covariance_pred[row, col];
    end for;
  end for;
end predict;
