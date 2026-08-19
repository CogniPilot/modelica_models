within Estimation.MocapExternalOdometryErrorState;
function correct "Pose correction for the mocap geometric error-state EKF"
  input Estimation.MocapExternalOdometryErrorState.State predicted;
  input Estimation.MocapExternalOdometryErrorState.PoseMeasurement measurement;
  input Estimation.MocapExternalOdometryErrorState.PoseMeasurementNoise measurementNoise;
  output Estimation.MocapExternalOdometryErrorState.State corrected;
  output Boolean accepted "True when the innovation system was positive definite";
protected
  MeasurementVector residual;
  MeasurementMatrix H;
  MeasurementCovariance R;
  CrossCovariance crossCovariance;
  MeasurementCovariance innovationCovariance;
  Real gainTranspose[6, 12];
  Gain gain;
  TangentVector correction;
  Covariance josephFactor;
  Covariance posteriorCovariance;
  Covariance resetJacobian;
  NominalState predictedNominal;
  NominalState correctedNominal;
algorithm
  predictedNominal := NominalState(
    attitude=predicted.attitude,
    velocity=predicted.velocity,
    position=predicted.position,
    angularVelocity=predicted.angularVelocity);
  residual := poseResidual(
    predictedNominal.attitude,
    predictedNominal.position,
    measurement.attitude,
    measurement.position);
  H := poseMeasurementMatrix();
  R := poseMeasurementCovariance(measurementNoise);

  crossCovariance := predicted.covariance * transpose(H);
  innovationCovariance := LinearAlgebra.symmetrize(
    H * crossCovariance + R);
  (gainTranspose, accepted) := LinearAlgebra.solveSPD(
    innovationCovariance,
    transpose(crossCovariance));

  if accepted then
    gain := transpose(gainTranspose);
    correction := gain * residual;
    correctedNominal := Estimation.MocapExternalOdometryErrorState.inject(
      NominalState(
        attitude=predicted.attitude,
        velocity=predicted.velocity,
        position=predicted.position,
        angularVelocity=predicted.angularVelocity),
      correction);
    corrected.attitude := correctedNominal.attitude;
    corrected.velocity := correctedNominal.velocity;
    corrected.position := correctedNominal.position;
    corrected.angularVelocity := correctedNominal.angularVelocity;
    josephFactor := identity(
      TangentLength) - gain * H;
    posteriorCovariance := LinearAlgebra.symmetrize(
      LinearAlgebra.josephUpdate(
        josephFactor,
        predicted.covariance,
        gain,
        R));
    resetJacobian := identity(TangentLength);
    resetJacobian[1:3, 1:3] :=
      LieGroups.SO3.Quat.right_jacobian(correction[1:3]);
    corrected.covariance := LinearAlgebra.symmetrize(
      resetJacobian * posteriorCovariance * transpose(resetJacobian));
  else
    corrected := predicted;
  end if;
end correct;
