within Estimation.StrapdownINS.UKF;

function correctMagnetometer
  "Unscented yaw-only correction from raw magnetic field"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Avionics.MagnetometerSample measurement;
  input Real magneticFieldWorldEnu_T[3];
  input Real innovationGate = 0.0;
  input Real measurementAge_s(unit = "s") = 0.0;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3] = zeros(3);
  input Real specificForceMeasuredBodyFlu_m_s2[3] = zeros(3);
  input Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  input Real maximumAidingDelay_s(unit = "s") = 0.25;
  output Estimation.StrapdownINS.UKF.State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real nominal[16];
  Real sigmaState[16];
  Real delayedSigmaState[16];
  Real sigma[TangentLength, SigmaCount];
  Boolean sigmaUsable;
  Boolean measurementUsable;
  Real referenceHeading;
  Real sigmaHeading;
  Real delayedEuler[3];
  Real measuredHeading;
  Real measuredHeadingVariance;
  Real sigmaMeasurement[1, SigmaCount];
  Real measured[1];
  Real measurementCovariance[1, 1];
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  delayedSigmaState := predictNominalVector(nominal,
    angularVelocityMeasuredBodyFlu_rad_s,
    specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
    -max(measurementAge_s, 0.0));
  delayedEuler := LieGroups.SO3.EulerB321.from_Quat(
    delayedSigmaState[7:10]);
  referenceHeading := delayedEuler[1];
  (measuredHeading, measuredHeadingVariance, measurementUsable) :=
    Estimation.StrapdownINS.magnetometerYawObservation(
      delayedSigmaState[7:10], measurement.magneticFieldBodyFlu_T,
      measurement.covarianceBody_T2, magneticFieldWorldEnu_T);
  for index in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, index]);
    delayedSigmaState := predictNominalVector(sigmaState,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
      -max(measurementAge_s, 0.0));
    delayedEuler := LieGroups.SO3.EulerB321.from_Quat(
      delayedSigmaState[7:10]);
    sigmaHeading := delayedEuler[1];
    sigmaMeasurement[1, index] := referenceHeading
      + MathUtilities.wrapAngle(sigmaHeading - referenceHeading);
  end for;
  measured[1] := referenceHeading
    + MathUtilities.wrapAngle(measuredHeading - referenceHeading);
  measurementCovariance[1, 1] := measuredHeadingVariance;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    corrected := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := Estimation.StrapdownINS.CorrectionRejectedTimestamp;
    normalizedInnovationSquared := 0.0;
  elseif not sigmaUsable or not measurementUsable then
    corrected := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason :=
      Estimation.StrapdownINS.CorrectionRejectedCovarianceUnusable;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctUnscented(predicted, sigmaMeasurement,
        measured, measurementCovariance, innovationGate);
  end if;
end correctMagnetometer;
