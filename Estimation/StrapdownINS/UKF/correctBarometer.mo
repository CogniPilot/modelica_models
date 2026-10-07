within Estimation.StrapdownINS.UKF;

function correctBarometer
  "Unscented vertical-position correction from bias-compensated pressure altitude"
  input State predicted;
  input Avionics.BarometerSample measurement;
  input Real barometerBias_m;
  input Real barometerBiasVariance_m2;
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
  Real nominal[16];
  Real sigmaState[16];
  Real delayedSigmaState[16];
  Real sigma[TangentLength, SigmaCount];
  Boolean sigmaUsable;
  Real sigmaMeasurement[1, SigmaCount];
  Real measured[1];
  Real measurementCovariance[1, 1];
  Real rotationWorldBody[3, 3];
  Real verticalDirectionLocal[3];
  Real verticalVariance_m2;
  Real covarianceFloor[TangentLength, TangentLength];
  Real floorIncrement_m2;
  State candidate;
  Boolean candidateAccepted;
  Integer candidateRejectionReason;
  Real candidateNis;
algorithm
  nominal := stateVector(predicted);
  (sigma, sigmaUsable) := sigmaTangents(predicted.covariance);
  for sigmaIndex in 1:SigmaCount loop
    sigmaState := injectVector(nominal, sigma[:, sigmaIndex]);
    delayedSigmaState := predictNominalVector(sigmaState,
      angularVelocityMeasuredBodyFlu_rad_s,
      specificForceMeasuredBodyFlu_m_s2, gravityWorldEnu_m_s2,
      -max(measurementAge_s, 0.0));
    sigmaMeasurement[1, sigmaIndex] := delayedSigmaState[3];
  end for;
  measured[1] := measurement.altitudeWorldEnu_m - barometerBias_m;
  measurementCovariance[1, 1] := measurement.variance_m2
    + max(barometerBiasVariance_m2, 0.0);
  candidate := State(
    positionWorldEnu_m=predicted.positionWorldEnu_m,
    velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
    quaternionWorldBody=predicted.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
    covariance=predicted.covariance);
  candidateAccepted := false;
  candidateNis := 0.0;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    candidateRejectionReason :=
      Estimation.StrapdownINS.CorrectionRejectedTimestamp;
  elseif not sigmaUsable then
    candidateRejectionReason :=
      Estimation.StrapdownINS.CorrectionRejectedFactorization;
  else
    (candidate, candidateAccepted, candidateRejectionReason, candidateNis) :=
      correctUnscented(predicted, sigmaMeasurement,
        measured, measurementCovariance, innovationGate);
  end if;
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(candidate.quaternionWorldBody);
  verticalDirectionLocal := rotationWorldBody[3, :];
  verticalVariance_m2 := verticalDirectionLocal
    * candidate.covariance[1:3, 1:3] * verticalDirectionLocal;
  floorIncrement_m2 := if candidateAccepted then
    max(barometerBiasVariance_m2 - verticalVariance_m2, 0.0) else 0.0;
  covarianceFloor := zeros(TangentLength, TangentLength);
  covarianceFloor[1:3, 1:3] := floorIncrement_m2
    * transpose({verticalDirectionLocal}) * {verticalDirectionLocal};
  corrected := State(
    positionWorldEnu_m=candidate.positionWorldEnu_m,
    velocityWorldEnu_m_s=candidate.velocityWorldEnu_m_s,
    quaternionWorldBody=candidate.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=candidate.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=candidate.accelerometerBiasBodyFlu_m_s2,
    covariance=LinearAlgebra.symmetrize(candidate.covariance + covarianceFloor));
  accepted := candidateAccepted;
  rejectionReason := candidateRejectionReason;
  normalizedInnovationSquared := candidateNis;
end correctBarometer;
