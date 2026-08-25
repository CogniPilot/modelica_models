within Estimation.StrapdownINS.ESKF;

function correctMocap "Correct position and attitude from motion capture"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.MocapSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real residual[6];
  Real H[6, TangentLength];
  Real measurementCovariance[6, 6];
  Real attitudeError[4];
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    predicted.quaternionWorldBody);
  attitudeError := LieGroups.SO3.Quat.product(
    LieGroups.SO3.Quat.inverse(predicted.quaternionWorldBody),
    LieGroups.SO3.Quat.normalize(measurement.quaternionWorldBody));
  residual := cat(1,
    transpose(rotationWorldBody)
      * (measurement.positionWorldEnu_m - predicted.positionWorldEnu_m),
    LieGroups.SO3.Quat.log_map(attitudeError));
  H := cat(1,
    cat(2, identity(3), zeros(3, TangentLength - 3)),
    cat(2, zeros(3, 6), identity(3),
      zeros(3, TangentLength - 9)));
  measurementCovariance := cat(1,
    cat(2, transpose(rotationWorldBody)
      * measurement.positionCovarianceWorld_m2 * rotationWorldBody,
      zeros(3, 3)),
    cat(2, zeros(3, 3), measurement.attitudeCovarianceBody_rad2));
  (corrected, accepted, rejectionReason, normalizedInnovationSquared) := correctLinear(
    predicted, residual, H, measurementCovariance, innovationGate);
end correctMocap;
