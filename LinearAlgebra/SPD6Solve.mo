within LinearAlgebra;
model SPD6Solve "Compatibility boundary for the packed six-state covariance solve"
  parameter Real pivot_floor = 1e-12;
  parameter Real symmetry_absolute = 1e-12;
  parameter Real symmetry_relative = 1e-8;
  input Real A[6,6] = identity(6);
  input Real B[6,16] = fill(0.0,6,16);
  output Real X[6,16];
  output Real valid;
protected
  Boolean accepted;
equation
  (X, accepted) = SymmetricMatrix.solveAbsolute(A, B, pivot_floor,
    symmetry_absolute, symmetry_relative);
  valid = if accepted then 1.0 else 0.0;
end SPD6Solve;
