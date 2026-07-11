within Tests.Assertions;
function spdResidual "Maximum residual of a successful SPD solve"
  input Real A[:, size(A, 1)];
  input Real B[size(A, 1), :];
  output Real residual;
protected
  Real X[size(B, 1), size(B, 2)];
  Boolean ok;
algorithm
  (X, ok) := LinearAlgebra.solveSPD(A, B);
  assert(ok, "solveSPD rejected a positive-definite test matrix");
  residual := Tests.Assertions.maxAbsMatrix(A * X - B);
end spdResidual;
