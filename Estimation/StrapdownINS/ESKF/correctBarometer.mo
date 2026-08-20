within Estimation.StrapdownINS.ESKF;

function correctBarometer
  "Correct vertical position from pressure altitude after bias compensation"
  input Estimation.StrapdownINS.ESKF.State predicted;
  input Avionics.BarometerSample measurement;
  input Real barometerBias_m;
  input Real barometerBiasVariance_m2;
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
  Real rotationWorldBody[3, 3];
  Real residual[1];
  Real delayedH[1, TangentLength];
  Real H[1, TangentLength];
  Real measurementCovariance[1, 1];
  Real verticalDirectionLocal[3];
  Real verticalVariance_m2;
  Real covarianceFloor[TangentLength, TangentLength];
  Real floorIncrement_m2;
  Estimation.StrapdownINS.ESKF.State candidate;
  Boolean candidateAccepted;
  Integer candidateRejectionReason;
  Real candidateNis;
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
  residual[1] := measurement.altitudeWorldEnu_m - barometerBias_m
    - delayedPosition[3];
  delayedH := zeros(1, TangentLength);
  delayedH[1, 1:3] := rotationWorldBody[3, :];
  H := delayedH * currentToDelayed;
  measurementCovariance[1, 1] := measurement.variance_m2
    + max(barometerBiasVariance_m2, 0.0);
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    candidate := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    candidateAccepted := false;
    candidateRejectionReason := CorrectionRejectedTimestamp;
    candidateNis := 0.0;
  elseif not delayAccepted then
    candidate := Estimation.StrapdownINS.ESKF.State(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2,
      covariance=predicted.covariance);
    candidateAccepted := false;
    candidateRejectionReason := CorrectionRejectedFactorization;
    candidateNis := 0.0;
  else
    (candidate, candidateAccepted, candidateRejectionReason, candidateNis) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate);
  end if;
  // The learned pressure datum is one common nuisance variable, not a new
  // independent error on every packet. Preserve its posterior uncertainty.
  verticalDirectionLocal := rotationWorldBody[3, :];
  verticalVariance_m2 := verticalDirectionLocal
    * candidate.covariance[1:3, 1:3] * verticalDirectionLocal;
  floorIncrement_m2 := if candidateAccepted then
    max(barometerBiasVariance_m2 - verticalVariance_m2, 0.0) else 0.0;
  covarianceFloor := zeros(TangentLength, TangentLength);
  for row in 1:3 loop
    for column in 1:3 loop
      covarianceFloor[row, column] := floorIncrement_m2
        * verticalDirectionLocal[row] * verticalDirectionLocal[column];
    end for;
  end for;
  corrected := Estimation.StrapdownINS.ESKF.State(
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
