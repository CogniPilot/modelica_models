within Estimation.MocapExternalOdometryErrorState;
function poseResidual "Invariant attitude and additive position innovation"
  input Quaternion predictedAttitude;
  input Vector3 predictedPosition;
  input Quaternion measuredAttitude;
  input Vector3 measuredPosition;
  output MeasurementVector residual;
protected
  Quaternion attitudeError;
algorithm
  attitudeError := LieGroups.SO3.Quat.product(
    LieGroups.SO3.Quat.inverse(predictedAttitude),
    LieGroups.SO3.Quat.normalize(measuredAttitude));
  residual[1:3] := LieGroups.SO3.Quat.log_map(attitudeError);
  residual[4:6] := measuredPosition - predictedPosition;
end poseResidual;
