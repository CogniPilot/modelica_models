within Estimation.MocapExternalOdometryIEKF;
function correct "Pose correction for the mocap IEKF"
  input Estimation.MocapExternalOdometryIEKF.State predicted;
  input Estimation.MocapExternalOdometryIEKF.PoseMeasurement measurement;
  input Estimation.MocapExternalOdometryIEKF.PoseMeasurementNoise measurementNoise;
  output Estimation.MocapExternalOdometryIEKF.State corrected;
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
    correctedNominal := Estimation.MocapExternalOdometryIEKF.inject(
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
    corrected.covariance := LinearAlgebra.symmetrize(
      LinearAlgebra.josephUpdate(
        josephFactor,
        predicted.covariance,
        gain,
        R));
  else
    corrected := predicted;
  end if;
end correct;
