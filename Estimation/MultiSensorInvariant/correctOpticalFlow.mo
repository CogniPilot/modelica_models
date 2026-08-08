within Estimation.MultiSensorInvariant;

function correctOpticalFlow
  "Correct planar body velocity from range-compensated optical flow"
  input Estimation.MultiSensorInvariant.State predicted;
  input Avionics.OpticalFlowSample measurement;
  output Estimation.MultiSensorInvariant.State corrected;
  output Boolean accepted;
protected
  Real rotationWorldBody[3, 3];
  Real predictedVelocityBody[3];
  Real velocityCross[3, 3];
  Real residual[2];
  Real H[2, TangentLength];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  predictedVelocityBody := transpose(rotationWorldBody)
    * predicted.velocityWorldEnu_m_s;
  residual := measurement.velocityBodyFlu_m_s
    - predictedVelocityBody[1:2];
  velocityCross := LieGroups.SO3.Quat.wedge(predictedVelocityBody);
  H := cat(2,
    zeros(2, 3),
    [1.0, 0.0, 0.0; 0.0, 1.0, 0.0],
    velocityCross[1:2, :],
    zeros(2, TangentLength - 9));
  (corrected, accepted) := correctLinear(
    predicted, residual, H, measurement.velocityCovarianceBody_m2_s2);
end correctOpticalFlow;
