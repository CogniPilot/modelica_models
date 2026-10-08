within Tests;

model CovarianceRootTests
protected
  constant Real rectangular[3, 5] =
    {{1, 2, 0, 0, 1}, {0, 3, 1, 0, 2}, {0, 0, 4, 1, 3}};
  constant Real rankDeficient[3, 2] = {{-2, 0}, {0, 3}, {1, 1}};
  Real root[3, 3];
  Real deficientRoot[3, 3];
  Real zeroRoot[3, 3];
equation
  root = LinearAlgebra.covarianceRoot(rectangular);
  deficientRoot = LinearAlgebra.covarianceRoot(rankDeficient);
  zeroRoot = LinearAlgebra.covarianceRoot(zeros(3, 1));
  assert(sum((root * transpose(root)
    - rectangular * transpose(rectangular)).^2) < 1.0e-20,
    "Rectangular covariance root changed the covariance");
  assert(abs(root[1, 2]) + abs(root[1, 3]) + abs(root[2, 3]) <= 0.0
      and min({root[1, 1], root[2, 2], root[3, 3]}) > 0.0,
    "Covariance root must be lower triangular with positive diagonal");
  assert(sum((deficientRoot * transpose(deficientRoot)
    - rankDeficient * transpose(rankDeficient)).^2) < 1.0e-20,
    "Rank-deficient covariance root changed the covariance");
  assert(sum(zeroRoot.^2) <= 0.0, "Zero covariance acquired artificial noise");
end CovarianceRootTests;
