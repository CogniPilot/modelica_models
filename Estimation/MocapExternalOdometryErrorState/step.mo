within Estimation.MocapExternalOdometryErrorState;
function step "One discrete geometric error-state EKF prediction and optional pose correction"
  input Estimation.MocapExternalOdometryErrorState.State previous;
  input Real dt "Prediction interval [s]";
  input Boolean measurementValid;
  input Estimation.MocapExternalOdometryErrorState.PoseMeasurement measurement;
  input Estimation.MocapExternalOdometryErrorState.ProcessNoise processNoise;
  input Estimation.MocapExternalOdometryErrorState.PoseMeasurementNoise measurementNoise;
  output Estimation.MocapExternalOdometryErrorState.State next;
  output Boolean correctionAccepted;
protected
  Estimation.MocapExternalOdometryErrorState.State predicted;
algorithm
  predicted := Estimation.MocapExternalOdometryErrorState.predict(
    previous,
    dt,
    processNoise);
  if measurementValid then
    (next, correctionAccepted) :=
      Estimation.MocapExternalOdometryErrorState.correct(
        predicted,
        PoseMeasurement(
          position=measurement.position,
          attitude=measurement.attitude),
        PoseMeasurementNoise(
          attitude=measurementNoise.attitude,
          position=measurementNoise.position));
  else
    next := predicted;
    correctionAccepted := false;
  end if;
end step;
