within LinearAlgebra;
function solveSPD
  "Solve A*X=B for symmetric positive-definite A using Cholesky"
  input Real A[:, size(A, 1)] "Symmetric positive-definite system matrix";
  input Real B[size(A, 1), :] "One or more right-hand sides";
  input Real relativeTolerance = 0.0
    "Optional extra floor on the Cholesky pivot threshold relative to the
     largest input diagonal; the working-precision term always applies";
  output Real X[size(B, 1), size(B, 2)] "Solution";
  output Boolean ok "False when A is not numerically positive definite";
algorithm
  (X, ok) := SymmetricMatrix.solveMatrix(
    SymmetricMatrix.packSymmetrized(A), B, relativeTolerance);
end solveSPD;
