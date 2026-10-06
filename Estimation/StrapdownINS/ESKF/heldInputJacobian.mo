within Estimation.StrapdownINS.ESKF;

function heldInputJacobian
  "Transport one held gyroscope/accelerometer sample into the local error"
  input Real A[TangentLength, TangentLength];
  input Real interval_s;
  output Real transport[TangentLength, 6];
protected
  Real fullNoiseInput[TangentLength, ProcessNoiseLength];
  Real G[TangentLength, 6];
  Real first[TangentLength, 6];
  Real second[TangentLength, 6];
  Real third[TangentLength, 6];
algorithm
  fullNoiseInput := noiseInputMatrix();
  G := fullNoiseInput[:, 1:6];
  first := interval_s * A * G;
  second := interval_s * A * first;
  third := interval_s * A * second;
  // Integrate the same third-order transition used by discreteTransition.
  // A negative interval transports the uncertainty of a held input backward.
  transport := interval_s * (G + 0.5 * first
    + (1.0 / 6.0) * second + (1.0 / 24.0) * third);
end heldInputJacobian;
