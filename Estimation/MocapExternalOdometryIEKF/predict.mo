within Estimation.MocapExternalOdometryIEKF;
function predict "Discrete SE_2(3) IEKF prediction"
  input Estimation.MocapExternalOdometryIEKF.State previous;
  input Real dt "Prediction interval [s]";
  input Estimation.MocapExternalOdometryIEKF.ProcessNoise processNoise;
  output Estimation.MocapExternalOdometryIEKF.State predicted;
protected
  NominalState previousNominal;
  NominalState predictedNominal;
  Covariance transition;
  Covariance discreteProcessNoise;
algorithm
  previousNominal := NominalState(
    attitude=previous.attitude,
    velocity=previous.velocity,
    position=previous.position,
    angularVelocity=previous.angularVelocity);
  predictedNominal := predictNominal(previousNominal, dt);
  predicted.attitude := predictedNominal.attitude;
  predicted.velocity := predictedNominal.velocity;
  predicted.position := predictedNominal.position;
  predicted.angularVelocity := predictedNominal.angularVelocity;
  transition := tangentTransition(dt, previous.angularVelocity);
  discreteProcessNoise := discreteProcessCovariance(dt, processNoise);

  predicted.covariance := LinearAlgebra.symmetrize(
    transition * previous.covariance * transpose(transition)
      + discreteProcessNoise);
end predict;
