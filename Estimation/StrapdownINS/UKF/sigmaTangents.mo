within Estimation.StrapdownINS.UKF;

function sigmaTangents
  "Generate 2*n+1 symmetric sigma points in the local tangent"
  input Covariance covariance;
  output Real sigma[TangentLength, SigmaCount];
  output Boolean success;
protected
  Real lower[div(TangentLength * (TangentLength + 1), 2)];
algorithm
  (lower, success) := lowerCholeskyPacked(
    LinearAlgebra.symmetrize(covariance));
  sigma := zeros(TangentLength, SigmaCount);
  for column in 1:TangentLength loop
    for row in column:TangentLength loop
      sigma[row, column + 1] := SigmaScale
        * lower[LinearAlgebra.LowerTriangular.index(row, column)];
      sigma[row, column + 1 + TangentLength] := -SigmaScale
        * lower[LinearAlgebra.LowerTriangular.index(row, column)];
    end for;
  end for;
end sigmaTangents;
