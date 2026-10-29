within Tests;
model PackedLinearAlgebraTests "Packed symmetric and triangular storage, solves and covariance operations"
  function runSizeCase "Sampled dense-residual checks retain one generic function body"
    input Integer n;
    input Real t;
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-10;
    Real A[n, n];
    Real B[n, 2];
    Real p[div(n * (n + 1), 2)];
    Real lower[div(n * (n + 1), 2)];
    Real upper[div(n * (n + 1), 2)];
    Real L[n, n];
    Real X[n, 2];
    Real Y[n, 2];
    Real Z[n, 2];
    Real transformed[3];
    Real qr[div(n * (n + 1), 2)];
    Boolean factorOk;
    Boolean solveOk;
    Boolean lowerOk;
    Boolean upperOk;
  algorithm
    A := {{if i == j then 2.0 + n + t else 1.0 / (1.0 + abs(i - j)) for j in 1:n} for i in 1:n};
    B := {{i + 0.5 * j for j in 1:2} for i in 1:n};
    p := LinearAlgebra.SymmetricMatrix.pack(A);
    (lower, factorOk) := LinearAlgebra.SymmetricMatrix.cholesky(n, p);
    L := LinearAlgebra.LowerTriangular.unpack(n, lower);
    upper := LinearAlgebra.LowerTriangular.transposePacked(n, lower);
    (X, solveOk) := LinearAlgebra.SymmetricMatrix.solveMatrix(p, B);
    (Y, lowerOk) := LinearAlgebra.LowerTriangular.solve(lower, B, true);
    (Z, upperOk) := LinearAlgebra.UpperTriangular.solve(upper, B, true);
    transformed := LinearAlgebra.SymmetricMatrix.transform(transpose(B), p);
    qr := LinearAlgebra.LowerTriangular.covarianceRoot(cat(2, L, zeros(n, 2)));
    assert(factorOk and solveOk and lowerOk and upperOk, "Packed SPD factor or triangular solve refused a valid system");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.unpack(n, p) - A) < tolerance, "Packed symmetric round trip failed");
    assert(Tests.Assertions.maxAbsMatrix(L * transpose(L) - A) < tolerance, "Packed Cholesky residual failed");
    assert(Tests.Assertions.maxAbsMatrix(A * X - B) < tolerance, "Packed SPD solve residual failed");
    assert(Tests.Assertions.maxAbsMatrix(L * Y - B) < tolerance, "Packed forward solve residual failed");
    assert(Tests.Assertions.maxAbsMatrix(transpose(L) * Z - B) < tolerance, "Packed backward solve residual failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.UpperTriangular.unpack(n, upper) - transpose(L)) < tolerance, "Packed transpose indexing failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.unpack(2, transformed) - transpose(B) * A * B) < tolerance, "Packed rectangular covariance transform failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.unpack(n, LinearAlgebra.LowerTriangular.covariance(n, qr)) - A) < tolerance, "Packed QR covariance root residual failed");
    assert(max(abs(LinearAlgebra.SymmetricMatrix.mulVec(p, B[:, 1]) - A * B[:, 1])) < tolerance, "Packed matrix-vector multiplication failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.mulMatrix(p, B) - A * B) < tolerance, "Packed matrix-column multiplication failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.unpack(n,
      LinearAlgebra.SymmetricMatrix.weightedOuterProduct(B[:, 1], 0.25))
        - transpose({0.25 * B[:, 1]}) * {B[:, 1]}) < tolerance, "Packed weighted rank-one update failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.unpack(n,
      LinearAlgebra.SymmetricMatrix.crossSum(B, 0.5 * B)) - B * transpose(B)) < tolerance, "Packed symmetric cross sum failed");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.SymmetricMatrix.unpack(2,
      LinearAlgebra.SymmetricMatrix.gram(B)) - transpose(B) * B) < tolerance, "Packed Gram matrix failed");
    passed := true;
  end runSizeCase;
  model SizeCase
    parameter Integer n=3;
    Boolean passed(start=true,fixed=true);
  algorithm
    when sample(0.0,0.01) then
      passed := runSizeCase(n,time);
    end when;
  equation
    assert(passed,"Packed size case did not pass");
  end SizeCase;
  function runChecks "Dense compatibility, refusal and MPC residual checks"
    input Real t;
    output Boolean passed;
  protected
    Real mpcJacobian[3,2];
    Real mpcStep[2];
    Boolean mpcAccepted;
    Real rejectedLower[3];
    Boolean indefiniteOk;
    Real semidefiniteLower[3];
    Boolean semidefiniteOk;
    Real invalidZeroPivot[3];
    Boolean invalidZeroPivotOk;
    Real asymmetricSolution[2,1];
    Boolean asymmetricOk;
    Real zeroPivotSolution[2,1];
    Boolean zeroPivotOk;
    Real negativePivotSolution[2,1];
    Boolean negativePivotOk;
  algorithm
    mpcJacobian := [1.0+t,0.2;0.1,1.0;0.5,-0.3];
    (mpcStep,mpcAccepted) := Control.Mpc.gaussNewtonSqpStep(mpcJacobian,{0.1,-0.2,0.05},0.01);
    assert(mpcAccepted and max(abs((transpose(mpcJacobian)*mpcJacobian
      + 0.01*identity(2))*mpcStep + transpose(mpcJacobian)*{0.1,-0.2,0.05})) < 1.0e-10,
      "Packed MPC step failed the damped normal-equation residual");
    (rejectedLower, indefiniteOk) := LinearAlgebra.SymmetricMatrix.cholesky(2, {1.0, 2.0, 1.0});
    (semidefiniteLower, semidefiniteOk) := LinearAlgebra.SymmetricMatrix.factorPSD(2, {1.0, 0.0, 0.0});
    (invalidZeroPivot, invalidZeroPivotOk) := LinearAlgebra.SymmetricMatrix.factorPSD(2, {0.0, 1.0, 1.0});
    (asymmetricSolution, asymmetricOk) := LinearAlgebra.SymmetricMatrix.solveAbsolute(
      [2.0,1.0;0.0,2.0], ones(2,1), 0.0, 1.0e-12, 1.0e-8);
    (zeroPivotSolution, zeroPivotOk) := LinearAlgebra.LowerTriangular.solve(
      {0.0,1.0,1.0}, ones(2,1), true);
    (negativePivotSolution, negativePivotOk) := LinearAlgebra.UpperTriangular.solve(
      {-1.0,0.0,1.0}, ones(2,1), true);
    assert(not asymmetricOk, "Absolute-pivot solve hid raw input asymmetry");
    assert(not zeroPivotOk and not negativePivotOk, "Triangular solve accepted an unusable pivot");
    assert(not indefiniteOk, "Packed Cholesky accepted an indefinite covariance");
    assert(semidefiniteOk and max(abs(semidefiniteLower - {1.0, 0.0, 0.0})) < 1.0e-10, "Packed semidefinite factor rejected a valid zero pivot");
    assert(not invalidZeroPivotOk, "Packed semidefinite factor accepted an inconsistent zero pivot");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.transformCovariance(identity(2),
      [1.0,2.0;0.0,1.0]) - [1.0,2.0;0.0,1.0]) < 1.0e-10,
      "Compatibility transform concealed an invalid antisymmetric covariance");
    assert(Tests.Assertions.maxAbsMatrix(LinearAlgebra.josephUpdate(identity(2),
      [1.0,2.0;0.0,1.0], zeros(2,1), [1.0]) - [1.0,2.0;0.0,1.0]) < 1.0e-10,
      "Compatibility Joseph update concealed an invalid antisymmetric covariance");
    passed := true;
  end runChecks;
  SizeCase one(n=1);
  SizeCase two(n=2);
  SizeCase three(n=3);
  SizeCase six(n=6);
  SizeCase twenty(n=20);
  Boolean passed(start=true,fixed=true);
algorithm
  when sample(0.0,0.01) then
    passed := runChecks(time);
  end when;
equation
  assert(passed,"Packed compatibility checks did not pass");
end PackedLinearAlgebraTests;
