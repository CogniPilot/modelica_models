within Estimation.MocapExternalOdometryErrorState;
function tangentTransition "Discrete tangent-state transition used by the geometric error-state EKF"
  input Real dt "Prediction interval [s]";
  input Vector3 angularVelocity = zeros(3)
    "Nominal body angular velocity [rad/s]";
  output Covariance transition;
protected
  Vector3 rotationIncrement;
  Quaternion incrementQuaternion;
algorithm
  assert(dt >= 0.0, "geometric error-state EKF prediction interval must be nonnegative");
  rotationIncrement := angularVelocity * dt;
  incrementQuaternion := LieGroups.SO3.Quat.exp_map(rotationIncrement);
  transition := identity(TangentLength);
  transition[1:3, 1:3] := transpose(
    LieGroups.SO3.Quat.to_DCM(incrementQuaternion));
  transition[1:3, 10:12] :=
    LieGroups.SO3.Quat.right_jacobian(rotationIncrement) * dt;
  transition[7:9, 4:6] := identity(3) * dt;
end tangentTransition;
