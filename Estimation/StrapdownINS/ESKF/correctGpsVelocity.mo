within Estimation.StrapdownINS.ESKF;

function correctGpsVelocity "Correct world velocity from GPS Doppler velocity"
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
    * (measurement.velocityWorldEnu_m_s - predicted.velocityWorldEnu_m_s);
  H := cat(2, zeros(3, 3), identity(3),
    zeros(3, TangentLength - 6));
  measurementCovariance := transpose(rotationWorldBody)
    * measurement.velocityCovarianceWorld_m2_s2 * rotationWorldBody;
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) := correctLinear(
    predicted, residual, H, measurementCovariance, innovationGate);
end correctGpsVelocity;
