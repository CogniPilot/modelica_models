within SLAM.Inertial;
// Compatibility boundary for the packed absolute-pivot covariance solve.
// Public dimensions and pivot/symmetry thresholds are unchanged.
function SchmidtCorrectionSolve
  input Real pivot_floor = 0.0;
  input Real symmetry_absolute = 1e-12;
  input Real symmetry_relative = 1e-8;
  input Real A[poseDimension,poseDimension] = identity(poseDimension);
  input Real B[poseDimension,rhsDimension] = fill(0.0,poseDimension,rhsDimension);
  output Real X[poseDimension,rhsDimension];
  output Real valid;
protected
  constant Integer poseDimension = 6;
  constant Integer rhsDimension = 16;
  Boolean accepted;
algorithm
  (X, accepted) := LinearAlgebra.SymmetricMatrix.solveAbsolute(A, B,
    pivot_floor, symmetry_absolute, symmetry_relative);
  valid := if accepted then 1.0 else 0.0;
end SchmidtCorrectionSolve;
