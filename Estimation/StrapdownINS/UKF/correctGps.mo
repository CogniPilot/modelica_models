within Estimation.StrapdownINS.UKF;

function correctGps "Unscented joint GPS position/velocity correction"
  input Estimation.StrapdownINS.UKF.State predicted;
  input Avionics.GpsSample measurement;
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
  Real sigmaMeasurement[6, SigmaCount];
  Real measured[6];
  Real covariance[6, 6];
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  for index in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, index]);
    delayedSigmaState := predictNominalVector(sigmaState,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
      -max(measurementAge_s, 0.0));
    sigmaMeasurement[:, index] := cat(1,
      delayedSigmaState[1:3], delayedSigmaState[4:6]);
  end for;
  measured := cat(1, measurement.positionWorldEnu_m,
    measurement.velocityWorldEnu_m_s);
  covariance := cat(1,
    cat(2, measurement.positionCovarianceWorld_m2, zeros(3, 3)),
    cat(2, zeros(3, 3), measurement.velocityCovarianceWorld_m2_s2));
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
  elseif not sigmaUsable then
    corrected := Estimation.StrapdownINS.UKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    accepted := false;
    rejectionReason := Estimation.StrapdownINS.CorrectionRejectedFactorization;
    normalizedInnovationSquared := 0.0;
  else
    (corrected, accepted, rejectionReason, normalizedInnovationSquared) :=
      correctUnscented(predicted, sigmaMeasurement, measured, covariance,
        innovationGate);
  end if;
end correctGps;
