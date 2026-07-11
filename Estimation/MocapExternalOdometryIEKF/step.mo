within Estimation.MocapExternalOdometryIEKF;
function step "One discrete IEKF prediction and optional pose correction"
  input Estimation.MocapExternalOdometryIEKF.State previous;
  input Real dt "Prediction interval [s]";
  input Boolean measurementValid;
  input Estimation.MocapExternalOdometryIEKF.PoseMeasurement measurement;
  input Estimation.MocapExternalOdometryIEKF.ProcessNoise processNoise;
  input Estimation.MocapExternalOdometryIEKF.PoseMeasurementNoise measurementNoise;
  output Estimation.MocapExternalOdometryIEKF.State next;
  output Boolean correctionAccepted;
protected
  Estimation.MocapExternalOdometryIEKF.State predicted;
algorithm
  predicted := Estimation.MocapExternalOdometryIEKF.predict(
    previous,
    dt,
    processNoise);
  if measurementValid then
    (next, correctionAccepted) :=
      Estimation.MocapExternalOdometryIEKF.correct(
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
