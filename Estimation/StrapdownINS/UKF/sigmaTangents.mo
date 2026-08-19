within Estimation.StrapdownINS.UKF;

function sigmaTangents
  "Generate 2*n+1 symmetric sigma points in the local tangent"
  input Estimation.StrapdownINS.UKF.Covariance covariance;
  output Real sigma[TangentLength, SigmaCount];
  output Boolean success;
protected
  Real lower[TangentLength, TangentLength];
algorithm
  (lower, success) := lowerCholesky(
    LinearAlgebra.symmetrize(covariance));
  sigma[:, 1] := zeros(TangentLength);
  for column in 1:TangentLength loop
    sigma[:, column + 1] := SigmaScale * lower[:, column];
    sigma[:, column + 1 + TangentLength] :=
      -SigmaScale * lower[:, column];
  end for;
end sigmaTangents;
