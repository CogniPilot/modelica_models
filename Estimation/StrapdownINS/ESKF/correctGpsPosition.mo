within Estimation.StrapdownINS.ESKF;

function correctGpsPosition "Correct world position from GPS"
  input State predicted;
  input Avionics.GpsSample measurement;
  input Real innovationGate = 0.0
    "Per-degree-of-freedom NIS gate; non-positive disables";
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  input Boolean useSemiDirectBias = false;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason
    "Estimation.StrapdownINS.Correction* outcome code";
  output Real normalizedInnovationSquared;
protected
  Real rotationWorldBody[3, 3];
  Real delayedPosition[3];
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Real residual[3];
  Real H[3, TangentLength];
  Real measurementCovariance[3, 3];
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
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(
    delayedQuaternion);
  residual := transpose(rotationWorldBody)
    * (measurement.positionWorldEnu_m - delayedPosition);
  H := cat(2, identity(3), zeros(3, TangentLength - 3))
    * currentToDelayed;
  measurementCovariance := transpose(rotationWorldBody)
    * measurement.positionCovarianceWorld_m2 * rotationWorldBody;
  corrected := copyState(predicted);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    rejectionReason := CorrectionRejectedTimestamp;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate, zeros(3), zeros(TangentLength, size(residual, 1)),
        false, useSemiDirectBias);
  end if;
end correctGpsPosition;
