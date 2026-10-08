within Estimation.StrapdownINS.ESKF;

function correctBarometer
  "Correct vertical position from pressure altitude after bias compensation"
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
  input Boolean useSemiDirectBias = false;
  input Boolean useBarometerBiasConsider = false;
  input Real barometerBiasProcessNoise_m2_s = 0.0;
  output State corrected;
  output Boolean accepted;
  output Integer rejectionReason;
  output Real normalizedInnovationSquared;
protected
  Real delayedPosition[3];
  Real delayedQuaternion[4];
  Real delayedStateVector[16];
  Real currentToDelayed[TangentLength, TangentLength];
  Real rotationWorldBody[3, 3];
  Real residual[1];
  Real delayedH[1, TangentLength];
  Real H[1, TangentLength];
  Real measurementCovariance[1, 1];
  Real measurementStateCrossCovariance[TangentLength, 1];
  Real measurementBarometerCrossCovariance[1];
  Real observationBiasVariance_m2;
  Real bias_m;
  Real biasVariance_m2;
  Real verticalDirectionLocal[3];
  Real verticalVariance_m2;
  Real covarianceFloor[TangentLength, TangentLength];
  Real floorIncrement_m2;
  Real verticalRoot[TangentLength];
  Real floorColumns[TangentLength, TangentLength + 1];
  State candidate;
  Boolean candidateAccepted;
  Integer candidateRejectionReason;
  Real candidateNis;
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
  bias_m := barometerBias_m;
  biasVariance_m2 := barometerBiasVariance_m2;
  if predicted.useJointBarometerBias then
    bias_m := predicted.barometerBias_m;
    biasVariance_m2 := predicted.barometerBiasVariance_m2;
  end if;
  residual[1] := measurement.altitudeWorldEnu_m - bias_m
    - delayedPosition[3];
  delayedH := zeros(1, TangentLength);
  delayedH[1, 1:3] := rotationWorldBody[3, :];
  H := delayedH * currentToDelayed;
  measurementCovariance[1, 1] := measurement.variance_m2
    + max(biasVariance_m2, 0.0);
  measurementStateCrossCovariance := zeros(TangentLength, 1);
  measurementBarometerCrossCovariance := zeros(1);
  observationBiasVariance_m2 := biasVariance_m2;
  if useBarometerBiasConsider or predicted.useJointBarometerBias then
    observationBiasVariance_m2 := biasVariance_m2
      - max(barometerBiasProcessNoise_m2_s, 0.0) * max(measurementAge_s, 0.0);
    measurementCovariance[1, 1] := measurement.variance_m2
      + observationBiasVariance_m2;
    measurementStateCrossCovariance :=
      transpose({predicted.barometerBiasCrossCovariance});
    measurementBarometerCrossCovariance[1] := observationBiasVariance_m2;
  end if;
  candidate := copyState(predicted);
  candidateAccepted := false;
  candidateNis := 0.0;
  if measurementAge_s < -1.0e-6
      or measurementAge_s > maximumAidingDelay_s then
    candidateRejectionReason := CorrectionRejectedTimestamp;
  elseif (useBarometerBiasConsider or predicted.useJointBarometerBias)
      and not (observationBiasVariance_m2 >= 0.0
      and observationBiasVariance_m2 < FiniteMagnitudeLimit) then
    candidateRejectionReason := CorrectionRejectedCovarianceUnusable;
  else
    (candidate, candidateAccepted, candidateRejectionReason, candidateNis) :=
      correctLinear(predicted, residual, H, measurementCovariance,
        innovationGate, zeros(3), measurementStateCrossCovariance,
        false, useSemiDirectBias, measurementBarometerCrossCovariance);
  end if;
  // The learned pressure datum is one common nuisance variable, not a new
  // independent error on every packet. Preserve its posterior uncertainty.
  verticalDirectionLocal := rotationWorldBody[3, :];
  if candidate.useSquareRootCovariance then
    verticalRoot := verticalDirectionLocal * candidate.covarianceRoot[1:3, :];
    verticalVariance_m2 := verticalRoot * verticalRoot;
  else
    verticalVariance_m2 := verticalDirectionLocal
      * candidate.covariance[1:3, 1:3] * verticalDirectionLocal;
  end if;
  floorIncrement_m2 := if candidateAccepted then
    max(barometerBiasVariance_m2 - verticalVariance_m2, 0.0) else 0.0;
  covarianceFloor := zeros(TangentLength, TangentLength);
  covarianceFloor[1:3, 1:3] := floorIncrement_m2
    * transpose({verticalDirectionLocal}) * {verticalDirectionLocal};
  if useBarometerBiasConsider or predicted.useJointBarometerBias then
    corrected := copyState(candidate);
  elseif candidate.useSquareRootCovariance then
    floorColumns := zeros(TangentLength, TangentLength + 1);
    floorColumns[:, 1:TangentLength] := candidate.covarianceRoot;
    floorColumns[:, TangentLength + 1] := cat(1,
      sqrt(floorIncrement_m2) * verticalDirectionLocal, zeros(TangentLength - 3));
    corrected := withCovarianceRoot(candidate,
      LinearAlgebra.covarianceRoot(floorColumns), candidate.barometerBiasCrossCovariance);
  else
    corrected := withDenseCovariance(candidate, LinearAlgebra.symmetrize(
      candidate.covariance + covarianceFloor), candidate.barometerBiasCrossCovariance);
  end if;
  accepted := candidateAccepted;
  rejectionReason := candidateRejectionReason;
  normalizedInnovationSquared := candidateNis;
end correctBarometer;
