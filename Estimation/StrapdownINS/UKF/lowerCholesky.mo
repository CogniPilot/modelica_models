within Estimation.StrapdownINS.UKF;

function lowerCholesky
  "Dense compatibility view of the precision-aware packed UKF factor"
  input Real covariance[TangentLength, TangentLength];
  output Real lower[TangentLength, TangentLength];
  output Boolean success;
protected
  Real packedLower[div(TangentLength * (TangentLength + 1), 2)];
algorithm
  (packedLower, success) := lowerCholeskyPacked(covariance);
  lower := LinearAlgebra.LowerTriangular.unpack(TangentLength, packedLower);
end lowerCholesky;
