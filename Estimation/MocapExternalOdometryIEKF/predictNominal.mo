within Estimation.MocapExternalOdometryIEKF;
function predictNominal
  "Pure discrete nominal-state map, separated from covariance propagation"
  input NominalState previous;
  input Real dt "Prediction interval [s]";
  output NominalState predicted;
protected
  Real se23State[10];
  Real leftIncrement[9];
  Real se23Prediction[10];
algorithm
  se23State := cat(1, previous.position, previous.velocity, previous.attitude);
  leftIncrement := cat(1, zeros(6), previous.angularVelocity * dt);
  se23Prediction := LieGroups.SE23.Quat.exp_mixed(
    se23State, leftIncrement, zeros(9), [0.0, dt; 0.0, 0.0]);
  predicted.position := se23Prediction[1:3];
  predicted.velocity := se23Prediction[4:6];
  predicted.attitude := LieGroups.SO3.Quat.normalize(se23Prediction[7:10]);
  predicted.angularVelocity := previous.angularVelocity;
end predictNominal;
