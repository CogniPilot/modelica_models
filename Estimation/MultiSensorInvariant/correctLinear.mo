within Estimation.MultiSensorInvariant;

function correctLinear
  "Vectorized Joseph-form correction in the invariant tangent space"
  input Estimation.MultiSensorInvariant.State predicted;
  input Real residual[:];
  input Real H[size(residual, 1), TangentLength];
  input Real measurementCovariance[size(residual, 1), size(residual, 1)];
  input Real innovationGate = 0.0
    "Reject when NIS exceeds innovationGate * size(residual, 1);
     non-positive disables the gate";
  output Estimation.MultiSensorInvariant.State corrected;
  output Boolean accepted;
  output Boolean gateRejected
    "True when the Cholesky solve succeeded but the innovation gate fired";
protected
  Integer measurementLength = size(residual, 1);
  Real crossCovariance[TangentLength, size(residual, 1)];
  Real innovationCovariance[size(residual, 1), size(residual, 1)];
  Real augmentedRhs[size(residual, 1), TangentLength + 1];
  Real augmentedSolution[size(residual, 1), TangentLength + 1];
  Real gainTranspose[size(residual, 1), TangentLength];
  Real gain[TangentLength, size(residual, 1)];
  Real normalizedInnovationSquared;
  Boolean factorized;
  Estimation.MultiSensorInvariant.TangentVector correction;
  Real josephFactor[TangentLength, TangentLength];
  Real posteriorCovariance[TangentLength, TangentLength];
  Real resetJacobian[TangentLength, TangentLength];
  Estimation.MultiSensorInvariant.NominalState nominal;
  Estimation.MultiSensorInvariant.NominalState correctedNominal;
  Estimation.MultiSensorInvariant.Covariance correctedCovariance;
algorithm
  crossCovariance := predicted.covariance * transpose(H);
  innovationCovariance := LinearAlgebra.symmetrize(
    H * crossCovariance + measurementCovariance);
  // One factorization serves both the gain solve S*K' = (P*H')' and the
  // whitened residual S^-1 * r needed for the innovation gate: append the
  // residual as one extra right-hand side.
  augmentedRhs := zeros(measurementLength, TangentLength + 1);
  augmentedRhs[:, 1:TangentLength] := transpose(crossCovariance);
  augmentedRhs[:, TangentLength + 1] := residual;
  (augmentedSolution, factorized) := LinearAlgebra.solveSPD(
    innovationCovariance, augmentedRhs);
  // Scalar normalized innovation squared (NIS) chi-square gate:
  // r' * S^-1 * r is chi-square distributed with size(residual, 1)
  // degrees of freedom when the filter is consistent, so a residual
  // grossly inconsistent with the predicted innovation covariance is
  // rejected even though S itself factors fine. The threshold is
  // expressed per degree of freedom so one configurable number covers
  // the 2-, 3-, and 6-dimensional corrections used here. The whitened
  // residual is all zeros when the factorization failed, so the gate
  // terms below are well defined on every path.
  normalizedInnovationSquared :=
    residual * augmentedSolution[:, TangentLength + 1];
  gateRejected := factorized and innovationGate > 0.0
    and normalizedInnovationSquared > innovationGate * measurementLength;
  accepted := factorized and not gateRejected;
  if accepted then
    gainTranspose := augmentedSolution[:, 1:TangentLength];
    gain := transpose(gainTranspose);
    correction := gain * residual;
    nominal := NominalState(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=predicted.accelerometerBiasBodyFlu_m_s2);
    correctedNominal := inject(nominal, correction);
    josephFactor := identity(TangentLength) - gain * H;
    posteriorCovariance := LinearAlgebra.symmetrize(
      LinearAlgebra.josephUpdate(
        josephFactor,
        predicted.covariance,
        gain,
        measurementCovariance));
    resetJacobian := cat(1,
      cat(2, LieGroups.SE23.Quat.right_jacobian(correction[1:9]),
        zeros(9, 6)),
      cat(2, zeros(6, 9), identity(6)));
    correctedCovariance := LinearAlgebra.symmetrize(
      resetJacobian * posteriorCovariance * transpose(resetJacobian));
  else
    // A rejected correction (factorization failure or gate) never
    // modifies the state.
    correctedNominal := Estimation.MultiSensorInvariant.NominalState(
      positionWorldEnu_m=predicted.positionWorldEnu_m,
      velocityWorldEnu_m_s=predicted.velocityWorldEnu_m_s,
      quaternionWorldBody=predicted.quaternionWorldBody,
      gyroscopeBiasBodyFlu_rad_s=predicted.gyroscopeBiasBodyFlu_rad_s,
      accelerometerBiasBodyFlu_m_s2=
        predicted.accelerometerBiasBodyFlu_m_s2);
    correctedCovariance := predicted.covariance;
  end if;
  corrected := Estimation.MultiSensorInvariant.State(
    positionWorldEnu_m=correctedNominal.positionWorldEnu_m,
    velocityWorldEnu_m_s=correctedNominal.velocityWorldEnu_m_s,
    quaternionWorldBody=correctedNominal.quaternionWorldBody,
    gyroscopeBiasBodyFlu_rad_s=
      correctedNominal.gyroscopeBiasBodyFlu_rad_s,
    accelerometerBiasBodyFlu_m_s2=
      correctedNominal.accelerometerBiasBodyFlu_m_s2,
    covariance=correctedCovariance);
end correctLinear;
