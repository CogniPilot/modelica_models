within Estimation.MultiSensorInvariant;

function correctLinear
  "Vectorized Joseph-form correction in the invariant tangent space"
  input Estimation.MultiSensorInvariant.State predicted;
  input Real residual[:];
  input Real H[size(residual, 1), TangentLength];
  input Real measurementCovariance[size(residual, 1), size(residual, 1)];
  output Estimation.MultiSensorInvariant.State corrected;
  output Boolean accepted;
protected
  Integer measurementLength = size(residual, 1);
  Real crossCovariance[TangentLength, size(residual, 1)];
  Real innovationCovariance[size(residual, 1), size(residual, 1)];
  Real gainTranspose[size(residual, 1), TangentLength];
  Real gain[TangentLength, size(residual, 1)];
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
  (gainTranspose, accepted) := LinearAlgebra.solveSPD(
    innovationCovariance, transpose(crossCovariance));
  if accepted then
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
