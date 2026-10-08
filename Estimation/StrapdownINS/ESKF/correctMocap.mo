within Estimation.StrapdownINS.ESKF;

function correctMocap "Correct position and attitude from motion capture"
  input State predicted;
  input Avionics.MocapSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real delayedPosition[3];
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Real rotationWorldBody[3, 3];
  Real residual[6];
  Real delayedH[6, TangentLength];
  Real H[6, TangentLength];
  Real measurementCovariance[6, 6];
  Real attitudeError[4];
algorithm
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedPosition := delayedStateVector[1:3];
  delayedQuaternion := delayedStateVector[7:10];
  currentToDelayed := discreteTransition(continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2),
    -max(measurementAge_s, 0.0));
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(delayedQuaternion);
  attitudeError := LieGroups.SO3.Quat.product(
    LieGroups.SO3.Quat.inverse(delayedQuaternion),
    LieGroups.SO3.Quat.normalize(measurement.quaternionWorldBody));
  residual := cat(1,
    transpose(rotationWorldBody)
      * (measurement.positionWorldEnu_m - delayedPosition),
    LieGroups.SO3.Quat.log_map(attitudeError));
  delayedH := cat(1,
    cat(2, identity(3), zeros(3, TangentLength - 3)),
    cat(2, zeros(3, 6), identity(3),
      zeros(3, TangentLength - 9)));
  H := delayedH * currentToDelayed;
  measurementCovariance := cat(1,
    cat(2, transpose(rotationWorldBody)
      * measurement.positionCovarianceWorld_m2 * rotationWorldBody,
      zeros(3, 3)),
    cat(2, zeros(3, 3), measurement.attitudeCovarianceBody_rad2));
  // A measurement the fusion instant has already passed, or one stamped in
  // the future, is REFUSED by timestamp rather than transported to meet the
  // state. The named outcome is what a supervisor can act on; a transported
  // one is an answer with an error nobody bounded.
  corrected := State(
    positionWorldEnu_m=predicted.positionWorldEnu_m,
    velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
    quaternionWorldBody=predicted.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
    covariance=predicted.covariance);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    rejectionReason := CorrectionRejectedTimestamp;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate);
  end if;
end correctMocap;
