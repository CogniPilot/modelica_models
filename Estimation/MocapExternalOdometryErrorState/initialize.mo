within Estimation.MocapExternalOdometryErrorState;
function initialize "Initialize the geometric error-state EKF from a pose measurement"
  input Real position[3] "Initial position in world ENU coordinates [m]";
  input Real attitude[4] "Initial scalar-first Hamilton quaternion";
  input Estimation.MocapExternalOdometryErrorState.InitialVariances variances;
  output Estimation.MocapExternalOdometryErrorState.State state;
algorithm
  state.attitude := LieGroups.SO3.Quat.normalize(attitude);
  state.velocity := zeros(3);
  state.position := position;
  state.angularVelocity := zeros(3);
  state.covariance := zeros(
    Estimation.MocapExternalOdometryErrorState.TangentLength,
    Estimation.MocapExternalOdometryErrorState.TangentLength);
  for axis in 1:3 loop
    state.covariance[axis, axis] := variances.attitude;
    state.covariance[axis + 3, axis + 3] := variances.velocity;
    state.covariance[axis + 6, axis + 6] := variances.position;
    state.covariance[axis + 9, axis + 9] := variances.angularVelocity;
  end for;
end initialize;
