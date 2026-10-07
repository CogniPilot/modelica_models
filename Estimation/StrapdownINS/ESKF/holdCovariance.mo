within Estimation.StrapdownINS.ESKF;

function holdCovariance
  "Grow covariance by one interval of process noise with the state held"
  input Covariance covariance;
  input Real dt(unit = "s");
  input Estimation.StrapdownINS.ProcessNoise processNoise;
  output Covariance grown;
protected
  Real A[TangentLength, TangentLength];
  Real G[TangentLength, ProcessNoiseLength];
  Real transition[TangentLength, TangentLength];
  Estimation.StrapdownINS.ProcessNoiseCovariance continuousNoise;
  Covariance discreteNoise;
algorithm
  A := continuousTransition(zeros(3), zeros(3));
  G := noiseInputMatrix();
  transition := discreteTransition(A, dt);
  continuousNoise := processNoiseMatrix(processNoise);
  discreteNoise := discreteProcessCovariance(A, G, continuousNoise, dt);
  grown := LinearAlgebra.symmetrize(
    transition * covariance * transpose(transition) + discreteNoise);
end holdCovariance;
