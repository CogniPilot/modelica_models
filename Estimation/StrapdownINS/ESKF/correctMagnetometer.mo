within Estimation.StrapdownINS.ESKF;

function correctMagnetometer
  "Tilt-compensate raw magnetic field and correct yaw only"
  input State predicted;
  input Avionics.MagnetometerSample measurement;
  input Real magneticFieldWorldEnu_T[3];
  input Real innovationGate = 0.0;
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Boolean measurementUsable;
  Real measuredHeading;
  Real measuredHeadingVariance;
  Real euler[3];
  Real cosPitch;
  Real delayedH[1, TangentLength];
  Real H[1, TangentLength];
  Real yawSensitivityBodyFlu[3];
  Real tiltSensitivityBodyFlu[3];
  Real predictedRotationWorldBody[3, 3];
  Real verticalAxisBodyFlu[3];
  Real residual[1];
  Real covariance[1, 1];
algorithm
  delayedStateVector := retrodict(predicted,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    max(measurementAge_s, 0.0));
  delayedQuaternion := delayedStateVector[7:10];
  currentToDelayed := discreteTransition(continuousTransition(
    angularVelocityMeasuredBodyFlu_rad_s
      - predicted.gyroscopeBiasBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2
      - predicted.accelerometerBiasBodyFlu_m_s2),
    -max(measurementAge_s, 0.0));
  (measuredHeading, measuredHeadingVariance, measurementUsable,
   yawSensitivityBodyFlu, tiltSensitivityBodyFlu) :=
    Estimation.StrapdownINS.magnetometerYawObservation(
      delayedQuaternion, measurement.magneticFieldBodyFlu_T,
      measurement.covarianceBody_T2, magneticFieldWorldEnu_T);
  euler := LieGroups.SO3.EulerB321.from_Quat(
    delayedQuaternion);
  cosPitch := cos(euler[2]);
  residual[1] := MathUtilities.wrapAngle(measuredHeading - euler[1]);
  delayedH := zeros(1, TangentLength);
  if abs(cosPitch) > 0.1 then
    delayedH[1, 7:9] := yawSensitivityBodyFlu + tiltSensitivityBodyFlu;
  end if;
  H := delayedH * currentToDelayed;
  // The gain acts in the CURRENT tangent, so the axis it is allowed to
  // rotate about is the world vertical resolved in the current body frame.
  predictedRotationWorldBody :=
    LieGroups.SO3.Quat.to_DCM(predicted.quaternionWorldBody);
  verticalAxisBodyFlu := predictedRotationWorldBody[3, :];
  covariance[1, 1] := measuredHeadingVariance;
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
  elseif not measurementUsable
      or abs(cosPitch) <= 0.1 then
    rejectionReason := CorrectionRejectedCovarianceUnusable;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctLinear(predicted, residual, H, covariance, innovationGate,
        verticalAxisBodyFlu);
  end if;
end correctMagnetometer;
