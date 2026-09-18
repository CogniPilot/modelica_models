within Estimation.StrapdownINS.ESKF;

function correctZeroVelocity
  "Fuse a synthetic measurement that the vehicle is not moving"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Real variance_m2_s2(unit = "m2/s2")
    "Per-axis measurement variance of the zero-velocity observation";
  input Real innovationGate = 0.0;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real residual[3];
  Real H[3, TangentLength];
  Real measurementCovariance[3, 3];
algorithm
  // Same body-frame velocity residual convention as correctGpsVelocity.
  // Through the velocity-attitude cross covariance this is the update that
  // makes a tilt error observable on a vehicle at rest with no aiding.
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  residual := -(transpose(rotationWorldBody) * predicted.velocityWorldEnu_m_s);
  H := cat(2, zeros(3, 3), identity(3), zeros(3, TangentLength - 6));
  measurementCovariance := identity(3) * variance_m2_s2;
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
    correctLinear(predicted, residual, H, measurementCovariance,
      innovationGate);
end correctZeroVelocity;
