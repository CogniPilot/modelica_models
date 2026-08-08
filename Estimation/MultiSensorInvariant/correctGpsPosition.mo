within Estimation.MultiSensorInvariant;

function correctGpsPosition "Correct world position from GPS"
  input Estimation.MultiSensorInvariant.State predicted;
  input Avionics.GpsSample measurement;
  output Estimation.MultiSensorInvariant.State corrected;
  output Boolean accepted;
protected
  Real rotationWorldBody[3, 3];
  Real residual[3];
  Real H[3, TangentLength];
  Real measurementCovariance[3, 3];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  residual := transpose(rotationWorldBody)
    * (measurement.positionWorldEnu_m - predicted.positionWorldEnu_m);
  H := cat(2, identity(3), zeros(3, TangentLength - 3));
  measurementCovariance := transpose(rotationWorldBody)
    * measurement.positionCovarianceWorld_m2 * rotationWorldBody;
  (corrected, accepted) := correctLinear(
    predicted, residual, H, measurementCovariance);
end correctGpsPosition;
