within Estimation.MocapExternalOdometryErrorState;
function predict "Discrete SE_2(3) geometric error-state EKF prediction"
  input Estimation.MocapExternalOdometryErrorState.State previous;
  input Real dt "Prediction interval [s]";
  input Estimation.MocapExternalOdometryErrorState.ProcessNoise processNoise;
  output Estimation.MocapExternalOdometryErrorState.State predicted;
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
