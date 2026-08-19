within Estimation.StrapdownINS.ESKF;

function correctGpsPosition "Correct world position from GPS"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.GpsSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
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
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) := correctLinear(
    predicted, residual, H, measurementCovariance, innovationGate);
end correctGpsPosition;
