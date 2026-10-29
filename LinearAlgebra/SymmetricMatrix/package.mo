within LinearAlgebra;
package SymmetricMatrix "Symmetric n x n matrices stored as their packed upper triangle (row-major); symmetry holds by construction"
  // Storage for size n: Real[SymmetricMatrix.packedSize(n)] = Real[div(n * (n + 1), 2)], entries
  // {a11, a12, ..., a1n, a22, ..., a2n, ..., ann}. Plain arrays and plain size-generic functions (no operator records,
  // no per-size copies), so they work as connector members, including flows (component-wise sums).

  function packedSize "Number of packed entries of a symmetric n x n matrix"
    input Integer n;
    output Integer m;
  algorithm
    m := div(n * (n + 1), 2);
  end packedSize;

  function index "Packed position of entry (i, j) of a symmetric n x n matrix"
    input Integer i;
    input Integer j;
    input Integer n;
    output Integer k;
  protected
    Integer r = min(i, j);
    Integer c = max(i, j);
  algorithm
    k := div((r - 1) * (2 * n - r + 2), 2) + c - r + 1;
  end index;

  function pack "Packed upper triangle of a square matrix (assumed symmetric)"
    input Real M[:, size(M, 1)];
    output Real p[div(size(M, 1) * (size(M, 1) + 1), 2)];
  protected
    Integer k;
  algorithm
    p := zeros(div(size(M, 1) * (size(M, 1) + 1), 2));
    k := 0;
    for i in 1:size(M, 1) loop
      for j in i:size(M, 1) loop
        k := k + 1;
        p[k] := M[i, j];
      end for;
    end for;
  end pack;

  function unpack "Full symmetric n x n matrix from its packed upper triangle"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real M[n, n];
  algorithm
    M := zeros(n, n);
    for i in 1:n loop
      for j in 1:n loop
        M[i, j] := p[index(i, j, n)];
      end for;
    end for;
  end unpack;

  function mulVec "y = S x, S packed, n = size(x)"
    input Real p[:];
    input Real x[:];
    output Real y[size(x, 1)];
  protected
    Integer n = size(x, 1);
  algorithm
    assert(size(p, 1) == div(n * (n + 1), 2), "SymmetricMatrix: packed length does not match n");
    y := zeros(n);
    for i in 1:n loop
      for j in 1:n loop
        y[i] := y[i] + p[index(i, j, n)] * x[j];
      end for;
    end for;
  end mulVec;

  function outerProduct "Packed u u' (symmetric rank one)"
    input Real u[:];
    output Real p[div(size(u, 1) * (size(u, 1) + 1), 2)];
  protected
    Integer k;
  algorithm
    p := zeros(div(size(u, 1) * (size(u, 1) + 1), 2));
    k := 0;
    for i in 1:size(u, 1) loop
      for j in i:size(u, 1) loop
        k := k + 1;
        p[k] := u[i] * u[j];
      end for;
    end for;
  end outerProduct;

  function weightedOuterProduct "Packed weight u u' with a prescribed per-entry multiplication order"
    input Real u[:];
    input Real weight;
    output Real p[div(size(u, 1) * (size(u, 1) + 1), 2)];
  algorithm
    p := zeros(div(size(u, 1) * (size(u, 1) + 1), 2));
    for i in 1:size(u, 1) loop
      for j in i:size(u, 1) loop
        p[index(i, j, size(u, 1))] := (weight * u[i]) * u[j];
      end for;
    end for;
  end weightedOuterProduct;

  function mulMatrix "Multiply a packed symmetric matrix by dense right-hand columns"
    input Real p[:];
    input Real B[:, :];
    output Real Y[size(B, 1), size(B, 2)];
  algorithm
    assert(size(p, 1) == packedSize(size(B, 1)), "SymmetricMatrix: packed length does not match n");
    Y := zeros(size(B, 1), size(B, 2));
    for i in 1:size(B, 1) loop
      for j in 1:size(B, 2) loop
        for k in 1:size(B, 1) loop
          Y[i, j] := Y[i, j] + p[index(i, k, size(B, 1))] * B[k, j];
        end for;
      end for;
    end for;
  end mulMatrix;

  function crossSum "Packed symmetric A B' + B A' for rectangular cross terms"
    input Real A[:, :];
    input Real B[size(A, 1), size(A, 2)];
    output Real p[div(size(A, 1) * (size(A, 1) + 1), 2)];
  protected
    Real ab;
    Real ba;
  algorithm
    p := zeros(div(size(A, 1) * (size(A, 1) + 1), 2));
    for i in 1:size(A, 1) loop
      for j in i:size(A, 1) loop
        ab := 0.0;
        ba := 0.0;
        for k in 1:size(A, 2) loop
          ab := ab + A[i, k] * B[j, k];
          ba := ba + B[i, k] * A[j, k];
        end for;
        p[index(i, j, size(A, 1))] := ab + ba;
      end for;
    end for;
  end crossSum;

  function congruence "Packed X' S X for a general n x n X and packed symmetric S"
    input Real X[:, size(X, 1)];
    input Real p[:];
    output Real q[div(size(X, 1) * (size(X, 1) + 1), 2)];
  protected
    Integer n = size(X, 1);
    Real SX[size(X, 1), size(X, 1)];
    Integer k;
  algorithm
    assert(size(p, 1) == div(n * (n + 1), 2), "SymmetricMatrix: packed length does not match n");
    SX := zeros(n, n);
    for j in 1:n loop
      SX[:, j] := mulVec(p, X[:, j]);
    end for;
    q := zeros(div(n * (n + 1), 2));
    k := 0;
    for i in 1:n loop
      for j in i:n loop
        k := k + 1;
        q[k] := X[:, i] * SX[:, j];
      end for;
    end for;
  end congruence;

  function solve "x = S^-1 b for a packed symmetric positive definite S (Cholesky); domain: S positive definite only"
    // Domain: S must be positive definite. Symmetric alone is not enough (an indefinite or singular S fails the
    // pivot assert below). Nothing else in this package assumes positive definiteness.
    input Real p[:];
    input Real b[:];
    output Real x[size(b, 1)];
  protected
    Integer n = size(b, 1);
    Real L[div(size(b, 1) * (size(b, 1) + 1), 2)];
    Real s;
  algorithm
    assert(size(p, 1) == div(n * (n + 1), 2), "SymmetricMatrix: packed length does not match n");
    L := zeros(div(n * (n + 1), 2));
    for j in 1:n loop
      s := p[index(j, j, n)];
      for k in 1:j - 1 loop
        s := s - L[LinearAlgebra.LowerTriangular.index(j, k)] ^ 2;
      end for;
      assert(s > 0, "SymmetricMatrix.solve: matrix is not positive definite");
      L[LinearAlgebra.LowerTriangular.index(j, j)] := sqrt(s);
      for i in j + 1:n loop
        s := p[index(i, j, n)];
        for k in 1:j - 1 loop
          s := s - L[LinearAlgebra.LowerTriangular.index(i, k)]
            * L[LinearAlgebra.LowerTriangular.index(j, k)];
        end for;
        L[LinearAlgebra.LowerTriangular.index(i, j)] := s
          / L[LinearAlgebra.LowerTriangular.index(j, j)];
      end for;
    end for;
    x := b;
    for i in 1:n loop
      for k in 1:i - 1 loop
        x[i] := x[i] - L[LinearAlgebra.LowerTriangular.index(i, k)] * x[k];
      end for;
      x[i] := x[i] / L[LinearAlgebra.LowerTriangular.index(i, i)];
    end for;
    for i in n:-1:1 loop
      for k in i + 1:n loop
        x[i] := x[i] - L[LinearAlgebra.LowerTriangular.index(k, i)] * x[k];
      end for;
      x[i] := x[i] / L[LinearAlgebra.LowerTriangular.index(i, i)];
    end for;
  end solve;

  function packSymmetrized "Average opposing dense entries into packed symmetric storage"
    input Real A[:, size(A, 1)];
    output Real p[div(size(A, 1) * (size(A, 1) + 1), 2)];
  algorithm
    p := zeros(div(size(A, 1) * (size(A, 1) + 1), 2));
    for i in 1:size(A, 1) loop
      for j in i:size(A, 1) loop
        p[index(i, j, size(A, 1))] := 0.5 * (A[i, j] + A[j, i]);
      end for;
    end for;
  end packSymmetrized;

  function cholesky "Packed lower Cholesky factor with working-precision scaled pivot acceptance"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    input Real relativeTolerance = 0.0 "May tighten, never weaken, the precision floor";
    output Real lower[div(n * (n + 1), 2)];
    output Boolean ok;
  protected
    Real scale;
    Real one;
    Real workingEpsilon;
    Real threshold;
    Real value;
  algorithm
    lower := zeros(div(n * (n + 1), 2));
    scale := 0.0;
    for i in 1:n loop
      scale := max(scale, abs(p[index(i, i, n)]));
    end for;
    scale := max(scale, 1.0e-30);
    one := scale / scale;
    workingEpsilon := 1.0;
    for precisionStep in 1:60 loop
      if one + 0.5 * workingEpsilon > one then
        workingEpsilon := 0.5 * workingEpsilon;
      end if;
    end for;
    threshold := max(relativeTolerance, n * workingEpsilon) * scale;
    ok := true;
    for i in 1:n loop
      for j in 1:i loop
        value := p[index(i, j, n)];
        for k in 1:j - 1 loop
          value := value - lower[LinearAlgebra.LowerTriangular.index(i, k)]
            * lower[LinearAlgebra.LowerTriangular.index(j, k)];
        end for;
        if i == j then
          if value > threshold then
            lower[LinearAlgebra.LowerTriangular.index(i, j)] := sqrt(value);
          else
            ok := false;
            lower[LinearAlgebra.LowerTriangular.index(i, j)] := 1.0;
          end if;
        else
          lower[LinearAlgebra.LowerTriangular.index(i, j)] := value
            / lower[LinearAlgebra.LowerTriangular.index(j, j)];
        end if;
      end for;
    end for;
  end cholesky;

  function solveMatrix "Solve a packed SPD system for multiple right-hand sides"
    input Real p[:];
    input Real B[:, :];
    input Real relativeTolerance = 0.0;
    output Real X[size(B, 1), size(B, 2)];
    output Boolean ok;
  protected
    Real lower[div(size(B, 1) * (size(B, 1) + 1), 2)];
    Real Y[size(B, 1), size(B, 2)];
    Boolean forwardOk;
    Boolean backwardOk;
  algorithm
    assert(size(p, 1) == packedSize(size(B, 1)), "SymmetricMatrix: packed length does not match n");
    (lower, ok) := cholesky(size(B, 1), p, relativeTolerance);
    X := zeros(size(B, 1), size(B, 2));
    if ok then
      (Y, forwardOk) := LinearAlgebra.LowerTriangular.solve(lower, B, true, 1.0e300);
      (X, backwardOk) := LinearAlgebra.UpperTriangular.solve(
        LinearAlgebra.LowerTriangular.transposePacked(size(B, 1), lower), Y, true, 1.0e300);
      ok := forwardOk and backwardOk;
    end if;
  end solveMatrix;

  function choleskyAbsolute "Packed factor with an explicit absolute pivot floor for compatibility callers"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    input Real pivotFloor;
    output Real lower[div(n * (n + 1), 2)];
    output Boolean ok;
  protected
    Real value;
  algorithm
    lower := zeros(div(n * (n + 1), 2));
    ok := true;
    for i in 1:n loop
      for j in 1:i loop
        value := p[index(i, j, n)];
        for k in 1:j - 1 loop
          value := value - lower[LinearAlgebra.LowerTriangular.index(i, k)]
            * lower[LinearAlgebra.LowerTriangular.index(j, k)];
        end for;
        if i == j then
          ok := ok and value > pivotFloor;
          lower[LinearAlgebra.LowerTriangular.index(i, j)] := sqrt(if value > pivotFloor then value else 1.0);
        else
          lower[LinearAlgebra.LowerTriangular.index(i, j)] := value
            / lower[LinearAlgebra.LowerTriangular.index(j, j)];
        end if;
      end for;
    end for;
  end choleskyAbsolute;

  function solveAbsolute "Packed SPD solve preserving a caller's absolute-pivot and symmetry guards"
    input Real A[:, size(A, 1)];
    input Real B[size(A, 1), :];
    input Real pivotFloor;
    input Real symmetryAbsolute;
    input Real symmetryRelative;
    output Real X[size(B, 1), size(B, 2)];
    output Boolean ok;
  protected
    Real lower[div(size(A, 1) * (size(A, 1) + 1), 2)];
    Real Y[size(B, 1), size(B, 2)];
    Boolean forwardOk;
    Boolean backwardOk;
  algorithm
    // Legacy SPD6 uses the lower triangle after checking both original sides;
    // averaging before this guard would conceal an asymmetric input.
    (lower, ok) := choleskyAbsolute(size(A, 1), pack(transpose(A)), pivotFloor);
    for i in 1:size(A, 1) loop
      for j in 1:size(A, 1) loop
        ok := ok and abs(A[i, j] - A[j, i]) <= symmetryAbsolute
          + symmetryRelative * abs(A[j, i]);
      end for;
    end for;
    X := zeros(size(B, 1), size(B, 2));
    if ok then
      (Y, forwardOk) := LinearAlgebra.LowerTriangular.solve(lower, B, true, 1.0e300);
      (X, backwardOk) := LinearAlgebra.UpperTriangular.solve(
        LinearAlgebra.LowerTriangular.transposePacked(size(A, 1), lower), Y, true, 1.0e300);
      ok := forwardOk and backwardOk;
    end if;
  end solveAbsolute;

  function factorPSD "Packed lower factor for a semidefinite covariance, preserving zero-pivot compatibility"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real lower[div(n * (n + 1), 2)];
    output Boolean ok;
  protected
    Real value;
  algorithm
    lower := zeros(div(n * (n + 1), 2));
    ok := true;
    for i in 1:n loop
      for j in 1:i loop
        value := p[index(i, j, n)];
        for k in 1:j - 1 loop
          value := value - lower[LinearAlgebra.LowerTriangular.index(i, k)]
            * lower[LinearAlgebra.LowerTriangular.index(j, k)];
        end for;
        if i == j then
          if value >= 0.0 and abs(value) < 1.0e30 then
            lower[LinearAlgebra.LowerTriangular.index(i, j)] := sqrt(value);
          else
            ok := false;
          end if;
        elseif lower[LinearAlgebra.LowerTriangular.index(j, j)] > 0.0 then
          lower[LinearAlgebra.LowerTriangular.index(i, j)] := value
            / lower[LinearAlgebra.LowerTriangular.index(j, j)];
        elseif not abs(value) <= 0.0 then
          ok := false;
        end if;
      end for;
    end for;
  end factorPSD;

  function isSymmetric "Exact dense symmetry check before choosing a packed compatibility path"
    input Real A[:, size(A, 1)];
    output Boolean symmetric;
  algorithm
    symmetric := true;
    for i in 1:size(A, 1) loop
      for j in i:size(A, 1) loop
        symmetric := symmetric and A[i, j] == A[j, i];
      end for;
    end for;
  end isSymmetric;

  function mulDenseVector "Packed symmetric multiplication with raw nonsymmetric-input compatibility"
    input Real covariance[:, size(covariance, 1)];
    input Real x[size(covariance, 1)];
    output Real y[size(x, 1)];
  algorithm
    y := if isSymmetric(covariance) then mulVec(pack(covariance), x)
      else covariance * x;
  end mulDenseVector;

  function mulDense "Dense compatibility multiply retaining nonsymmetric inputs for downstream validation"
    input Real covariance[:, size(covariance, 1)];
    input Real B[size(covariance, 1), :];
    output Real result[size(B, 1), size(B, 2)];
  algorithm
    result := if isSymmetric(covariance) then mulMatrix(pack(covariance), B)
      else covariance * B;
  end mulDense;

  function transformDense "Dense compatibility result without hiding a nonsymmetric input from downstream guards"
    input Real T[:, :];
    input Real covariance[size(T, 2), size(T, 2)];
    output Real result[size(T, 1), size(T, 1)];
  protected
    Real product[size(T, 1), size(T, 2)];
  algorithm
    if isSymmetric(covariance) then
      result := unpack(size(T, 1), transform(T, pack(covariance)));
    else
      // Legacy callers can pass unvalidated matrices. Their antisymmetric
      // part must reach the original downstream symmetry/refusal checks.
      product := T * covariance;
      result := product * transpose(T);
    end if;
  end transformDense;

  function josephUpdateDense "Dense compatibility wrapper preserving nonsymmetric-input behavior"
    input Real factor[:, size(factor, 1)];
    input Real covariance[size(factor, 1), size(factor, 1)];
    input Real gain[size(factor, 1), :];
    input Real noise[size(gain, 2), size(gain, 2)];
    output Real result[size(factor, 1), size(factor, 1)];
  algorithm
    if isSymmetric(covariance) and isSymmetric(noise) then
      result := unpack(size(factor, 1), josephUpdate(factor, pack(covariance), gain, pack(noise)));
    else
      result := transformDense(factor, covariance) + transformDense(gain, noise);
    end if;
  end josephUpdateDense;

  function gram "Packed A' A from dense rectangular columns"
    input Real A[:, :];
    output Real p[div(size(A, 2) * (size(A, 2) + 1), 2)];
  protected
    Real value;
  algorithm
    p := zeros(div(size(A, 2) * (size(A, 2) + 1), 2));
    for i in 1:size(A, 2) loop
      for j in i:size(A, 2) loop
        value := 0.0;
        for k in 1:size(A, 1) loop
          value := value + A[k, i] * A[k, j];
        end for;
        p[index(i, j, size(A, 2))] := value;
      end for;
    end for;
  end gram;

  function transform "Packed T P T' for a rectangular linear map T"
    input Real T[:, :];
    input Real p[div(size(T, 2) * (size(T, 2) + 1), 2)];
    output Real q[div(size(T, 1) * (size(T, 1) + 1), 2)];
  protected
    Real TP[size(T, 1), size(T, 2)];
    Real value;
  algorithm
    q := zeros(div(size(T, 1) * (size(T, 1) + 1), 2));
    TP := zeros(size(T, 1), size(T, 2));
    for i in 1:size(T, 1) loop
      for j in 1:size(T, 2) loop
        for k in 1:size(T, 2) loop
          TP[i, j] := TP[i, j] + T[i, k] * p[index(k, j, size(T, 2))];
        end for;
      end for;
    end for;
    for i in 1:size(T, 1) loop
      for j in i:size(T, 1) loop
        value := 0.0;
        for k in 1:size(T, 2) loop
          value := value + TP[i, k] * T[j, k];
        end for;
        q[index(i, j, size(T, 1))] := value;
      end for;
    end for;
  end transform;

  function josephUpdate "Joseph covariance update with packed covariance and noise storage"
    input Real factor[:, size(factor, 1)];
    input Real p[div(size(factor, 1) * (size(factor, 1) + 1), 2)];
    input Real gain[size(factor, 1), :];
    input Real noise[div(size(gain, 2) * (size(gain, 2) + 1), 2)];
    output Real q[div(size(factor, 1) * (size(factor, 1) + 1), 2)];
  algorithm
    q := transform(factor, p) + transform(gain, noise);
  end josephUpdate;
  annotation(Documentation(info="<html>
<p>Size-generic symmetric matrices store the upper triangle row by row:
{a11, a12, ..., a1n, a22, ..., ann}. The storage length is n(n+1)/2.
<code>index(i,j,n)</code> accepts either order of i and j; symmetry is structural.</p>
<p><code>pack</code> assumes symmetry and copies the upper side;
<code>packSymmetrized</code> explicitly averages the two original sides.
Neither operation validates positive definiteness. Dense compatibility helpers
check exact symmetry before selecting packed arithmetic; nonsymmetric inputs
retain dense multiplication so downstream validators can see the original defect.</p>
<p><code>cholesky</code> and <code>solveMatrix</code> return an affirmative Boolean
and retain a working-precision pivot floor n*epsilon*max(abs(diagonal)); an
extra relative tolerance may only tighten it. The factor is not usable when
that Boolean is false. <code>solve</code> is the assert-only SPD vector API.
<code>factorPSD</code> additionally permits a consistent zero pivot.
<code>solveAbsolute</code> preserves raw symmetry checks and an explicit pivot
floor for legacy fixed-size callers; it does not average away asymmetry.</p>
<p>Lower factors use <a href=\"modelica://LinearAlgebra.LowerTriangular\">LowerTriangular</a>
row-major storage. <code>transform</code>, <code>gram</code>, rank-one and Joseph
updates produce packed symmetric outputs. Rectangular cross covariance and
state-transition matrices remain dense. Existing public covariance records and
matrix-valued signatures remain dense compatibility views; packed source
storage alone does not establish generated memory or throughput savings.</p>
</html>"));
end SymmetricMatrix;
