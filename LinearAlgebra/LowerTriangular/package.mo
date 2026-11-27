within LinearAlgebra;
package LowerTriangular "Size-generic lower triangular matrices in packed row-major storage"
  function packedSize "Number of stored entries of an n by n triangular matrix"
    input Integer n;
    output Integer m;
  algorithm
    m := div(n * (n + 1), 2);
  end packedSize;

  function index "Packed index for a lower-triangular entry, i >= j"
    input Integer i;
    input Integer j;
    output Integer k;
  algorithm
    k := div(i * (i - 1), 2) + j;
  end index;

  function pack "Store the lower triangle of a square matrix"
    input Real A[:, size(A, 1)];
    output Real p[div(size(A, 1) * (size(A, 1) + 1), 2)];
  algorithm
    p := zeros(div(size(A, 1) * (size(A, 1) + 1), 2));
    for i in 1:size(A, 1) loop
      for j in 1:i loop
        p[index(i, j)] := A[i, j];
      end for;
    end for;
  end pack;

  function unpack "Expand packed lower storage with structural zeros above the diagonal"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real A[n, n];
  algorithm
    A := zeros(n, n);
    for i in 1:n loop
      for j in 1:i loop
        A[i, j] := p[index(i, j)];
      end for;
    end for;
  end unpack;

  function transposePacked "Transpose a packed lower factor into packed upper storage"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real q[div(n * (n + 1), 2)];
  algorithm
    q := zeros(div(n * (n + 1), 2));
    for i in 1:n loop
      for j in i:n loop
        q[LinearAlgebra.UpperTriangular.index(i, j, n)] := p[index(j, i)];
      end for;
    end for;
  end transposePacked;

  function mul "Multiply a packed lower matrix by one or more dense columns"
    input Real p[:];
    input Real B[:, :];
    output Real Y[size(B, 1), size(B, 2)];
  algorithm
    assert(size(p, 1) == packedSize(size(B, 1)), "LowerTriangular: packed length does not match n");
    Y := zeros(size(B, 1), size(B, 2));
    for i in 1:size(B, 1) loop
      for j in 1:size(B, 2) loop
        for k in 1:i loop
          Y[i, j] := Y[i, j] + p[index(i, k)] * B[k, j];
        end for;
      end for;
    end for;
  end mul;

  function solve "Forward substitution on packed storage, with explicit pivot acceptance"
    input Real p[:];
    input Real B[:, :];
    input Boolean positiveDiagonal = false "Require positive rather than nonzero diagonal";
    input Real finiteMagnitudeLimit = 1.0e30;
    output Real X[size(B, 1), size(B, 2)];
    output Boolean ok;
  protected
    Real value;
    Real pivot;
    Boolean usable;
  algorithm
    assert(size(p, 1) == packedSize(size(B, 1)), "LowerTriangular: packed length does not match n");
    X := zeros(size(B, 1), size(B, 2));
    ok := true;
    for i in 1:size(B, 1) loop
      pivot := p[index(i, i)];
      usable := (if positiveDiagonal then pivot > 0.0 else abs(pivot) > 0.0)
        and abs(pivot) < finiteMagnitudeLimit;
      ok := ok and usable;
      for j in 1:size(B, 2) loop
        value := B[i, j];
        for k in 1:i - 1 loop
          value := value - p[index(i, k)] * X[k, j];
        end for;
        X[i, j] := if usable then value / pivot else 0.0;
      end for;
    end for;
  end solve;

  function covariance "Packed symmetric L L' without expanding the triangular factor"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real q[div(n * (n + 1), 2)];
  protected
    Real value;
  algorithm
    q := zeros(div(n * (n + 1), 2));
    for i in 1:n loop
      for j in i:n loop
        value := 0.0;
        for k in 1:i loop
          value := value + p[index(i, k)] * p[index(j, k)];
        end for;
        q[LinearAlgebra.SymmetricMatrix.index(i, j, n)] := value;
      end for;
    end for;
  end covariance;

  function covarianceRoot "Packed lower covariance root from factor columns using Givens QR"
    input Real columns[:, :];
    output Real root[div(size(columns, 1) * (size(columns, 1) + 1), 2)];
  protected
    Real work[size(columns, 2), size(columns, 1)];
    Real upperRow[size(columns, 1)];
    Real scale;
    Real radius;
    Real cosine;
    Real sine;
  algorithm
    root := zeros(div(size(columns, 1) * (size(columns, 1) + 1), 2));
    // A failed QR on NaN columns need not be triangular. Reject such inputs
    // before discarding the structural upper half. Dense legacy callers
    // retain their original diagnostic propagation in covarianceRoot.
    for row in 1:size(columns, 1) loop
      for column in 1:size(columns, 2) loop
        assert(abs(columns[row, column]) < 1.0e300,
          "Packed covarianceRoot requires finite factor columns below 1e300");
      end for;
    end for;
    work := transpose(columns);
    for column in 1:size(columns, 1) loop
      for row in column + 1:size(work, 1) loop
        scale := max(abs(work[column, column]), abs(work[row, column]));
        if scale > 0.0 then
          radius := sqrt((work[column, column] / scale)^2 + (work[row, column] / scale)^2);
          cosine := (work[column, column] / scale) / radius;
          sine := (work[row, column] / scale) / radius;
          upperRow := {work[column, entry] for entry in 1:size(columns, 1)};
          work[column, :] := cosine * upperRow + sine * work[row, :];
          work[row, :] := -sine * upperRow + cosine * work[row, :];
          work[column, column] := scale * radius;
          work[row, column] := 0.0;
        end if;
      end for;
    end for;
    for i in 1:size(columns, 1) loop
      for j in 1:i loop
        root[index(i, j)] := if j > size(work, 1) then 0.0
          elseif work[j, j] < 0.0 then -work[j, i] else work[j, i];
      end for;
    end for;
  end covarianceRoot;
  annotation(Documentation(info="<html>
<p>Lower factors store {l11, l21, l22, l31, l32, l33, ...};
index(i,j) requires 1 &lt;= j &lt;= i &lt;= n. Upper structural zeros are not stored.
pack/unpack and transposePacked convert at dense compatibility boundaries.</p>
<p>solve accepts multiple right-hand sides. Its Boolean reports the explicit
nonzero (or positive) diagonal and magnitude-limit checks. The default limit
is 1e30; compatibility SPD wrappers use 1e300 because their factors are square
roots of finite Real pivots. Rejected factors must not be treated as solutions.</p>
<p>covariance forms packed L*L′. covarianceRoot uses the original Givens QR
order and requires finite input columns below 1e300. The legacy dense root
wrapper retains its full exceptional-input path outside its packed-safe domain.</p>
</html>"));
end LowerTriangular;
