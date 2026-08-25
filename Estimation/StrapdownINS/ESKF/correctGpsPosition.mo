within Estimation.StrapdownINS.ESKF;

function correctGpsPosition "Correct world position from GPS"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.GpsSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real delayedPosition[3];
  Real delayedVelocity[3];
  Real delayedQuaternion[4];
  Real delayedGyroscopeBias[3];
  Real delayedAccelerometerBias[3];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Boolean delayAccepted;
  Real residual[3];
  Real H[3, TangentLength];
  Real measurementCovariance[3, 3];
algorithm
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedPosition := delayedStateVector[1:3];
  delayedVelocity := delayedStateVector[4:6];
  delayedQuaternion := delayedStateVector[7:10];
  delayedGyroscopeBias := delayedStateVector[11:13];
  delayedAccelerometerBias := delayedStateVector[14:16];
  currentToDelayed := discreteTransition(continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2),
    -max(measurementAge_s, 0.0));
  delayAccepted := true;
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    delayedQuaternion);
  residual := transpose(rotationWorldBody)
    * (measurement.positionWorldEnu_m - delayedPosition);
  H := cat(2, identity(3), zeros(3, TangentLength - 3))
    * currentToDelayed;
  measurementCovariance := transpose(rotationWorldBody)
    * measurement.positionCovarianceWorld_m2 * rotationWorldBody;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedTimestamp;
    normalizedInnovationSquared := 0.0;
  elseif not delayAccepted then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedFactorization;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate);
  end if;
end correctGpsPosition;
