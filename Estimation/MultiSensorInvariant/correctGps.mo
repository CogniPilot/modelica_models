within Estimation.MultiSensorInvariant;

function correctGps "Jointly correct GPS position and velocity"
  input Estimation.MultiSensorInvariant.State predicted;
  input Avionics.GpsSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  output Estimation.MultiSensorInvariant.State corrected;
  output Boolean accepted;
  output Boolean gateRejected;
protected
  Real rotationWorldBody[3, 3];
  Real residual[6];
  Real H[6, TangentLength];
  Real measurementCovariance[6, 6];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  residual := cat(1,
    transpose(rotationWorldBody)
      * (measurement.positionWorldEnu_m - predicted.positionWorldEnu_m),
    transpose(rotationWorldBody)
      * (measurement.velocityWorldEnu_m_s - predicted.velocityWorldEnu_m_s));
  H := cat(1,
    cat(2, identity(3), zeros(3, TangentLength - 3)),
    cat(2, zeros(3, 3), identity(3),
      zeros(3, TangentLength - 6)));
  measurementCovariance := cat(1,
    cat(2, transpose(rotationWorldBody)
      * measurement.positionCovarianceWorld_m2 * rotationWorldBody,
      zeros(3, 3)),
    cat(2, zeros(3, 3), transpose(rotationWorldBody)
      * measurement.velocityCovarianceWorld_m2_s2 * rotationWorldBody));
  (corrected, accepted, gateRejected) := correctLinear(
    predicted, residual, H, measurementCovariance, innovationGate);
end correctGps;
