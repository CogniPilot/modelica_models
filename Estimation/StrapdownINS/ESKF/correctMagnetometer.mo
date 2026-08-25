within Estimation.StrapdownINS.ESKF;

function correctMagnetometer
  "Tilt-compensate raw magnetic field and correct yaw only"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.MagnetometerSample measurement;
  input Real magneticFieldWorldEnu_T[3];
  input Real innovationGate = 0.0;
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output Estimation.StrapdownINS.ESKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real delayedPosition[3];
  Real delayedVelocity[3];
  Real delayedQuaternion[4];
  Real delayedGyroscopeBias[3];
  Real delayedAccelerometerBias[3];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Boolean delayAccepted;
  Boolean measurementUsable;
  Real measuredHeading;
  Real measuredHeadingVariance;
  Real euler[3];
  Real cosPitch;
  Real delayedH[1, TangentLength];
  Real H[1, TangentLength];
  Real residual[1];
  Real covariance[1, 1];
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
  (measuredHeading, measuredHeadingVariance, measurementUsable) :=
    Estimation.StrapdownINS.magnetometerYawObservation(
      delayedQuaternion, measurement.magneticFieldBodyFlu_T,
      measurement.covarianceBody_T2, magneticFieldWorldEnu_T);
  euler := LieGroups.SO3.EulerB321.from_Quat(
    delayedQuaternion);
  cosPitch := cos(euler[2]);
  residual[1] := MathUtilities.wrapAngle(measuredHeading - euler[1]);
  delayedH := zeros(1, TangentLength);
  if abs(cosPitch) > 0.1 then
    // Only the yaw row is observable. Magnetometer data cannot directly
    // change roll or pitch, matching PX4's magnetic-heading fusion mode.
    delayedH[1, 7] := 0.0;
    delayedH[1, 8] := sin(euler[3]) / cosPitch;
    delayedH[1, 9] := cos(euler[3]) / cosPitch;
  end if;
  H := delayedH * currentToDelayed;
  covariance[1, 1] := measuredHeadingVariance;
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
  elseif not delayAccepted or not measurementUsable
      or abs(cosPitch) <= 0.1 then
    corrected := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := CorrectionRejectedCovarianceUnusable;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, covariance, innovationGate);
  end if;
end correctMagnetometer;
