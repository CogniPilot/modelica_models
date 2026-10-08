within Estimation.StrapdownINS.ESKF;

function correctMagnetometer
  "Correct heading or a calibrated magnetic vector in the local tangent"
  input State predicted;
  input Avionics.MagnetometerSample measurement;
  input Real magneticFieldWorldEnu_T[3];
  input Real innovationGate = 0.0;
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  input Boolean useEquivariantVector = false
    "Fuse a calibrated local field vector instead of heading alone";
  input Boolean useSemiDirectBias = false;
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
  Real vectorResidual[3];
  Real vectorAttitudeJacobian[3, 3];
  Real vectorCovariance[3, 3];
  Real vectorH[3, TangentLength];
  Boolean vectorUsable;
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
  corrected := copyState(predicted);
  accepted := false;
  normalizedInnovationSquared := 0.0;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    rejectionReason := CorrectionRejectedTimestamp;
  elseif useEquivariantVector then
    (vectorResidual, vectorAttitudeJacobian, vectorCovariance, vectorUsable) :=
      Estimation.StrapdownINS.magnetometerVectorObservation(
        delayedQuaternion, measurement.magneticFieldBodyFlu_T,
        measurement.covarianceBody_T2, magneticFieldWorldEnu_T);
    vectorH := cat(2, zeros(3, 6), vectorAttitudeJacobian, zeros(3, 6))
      * currentToDelayed;
    if vectorUsable then
      (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
        correctLinear(predicted, vectorResidual, vectorH, vectorCovariance,
          innovationGate, zeros(3), zeros(TangentLength, 3), false,
          useSemiDirectBias);
    else
      rejectionReason := CorrectionRejectedCovarianceUnusable;
    end if;
  else
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
    if not measurementUsable or abs(cosPitch) <= 0.1 then
      rejectionReason := CorrectionRejectedCovarianceUnusable;
    else
      (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
        correctLinear(predicted, residual, H, covariance, innovationGate,
          verticalAxisBodyFlu, zeros(TangentLength, 1), true,
          useSemiDirectBias);
    end if;
  end if;
end correctMagnetometer;
