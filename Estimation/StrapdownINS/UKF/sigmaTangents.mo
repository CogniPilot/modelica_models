within Estimation.StrapdownINS.UKF;

function sigmaTangents
  "Generate 2*n+1 symmetric sigma points in the local tangent"
  input Covariance covariance;
  output Real sigma[TangentLength, SigmaCount];
  output Boolean success;
protected
  Real lower[TangentLength, TangentLength];
algorithm
  (lower, success) := lowerCholesky(
    LinearAlgebra.symmetrize(covariance));
  sigma := cat(2, zeros(TangentLength, 1),
    SigmaScale * lower, -SigmaScale * lower);
end sigmaTangents;
