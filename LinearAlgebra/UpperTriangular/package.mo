within LinearAlgebra;
package UpperTriangular "Size-generic upper triangular matrices in packed row-major storage"
  function packedSize "Number of stored entries of an n by n triangular matrix"
    input Integer n;
    output Integer m;
  algorithm
    m := div(n * (n + 1), 2);
  end packedSize;

  function index "Packed index for an upper-triangular entry, i <= j"
    input Integer i;
    input Integer j;
    input Integer n;
    output Integer k;
  algorithm
    k := div((i - 1) * (2 * n - i + 2), 2) + j - i + 1;
  end index;

  function pack "Store the upper triangle of a square matrix"
    input Real A[:, size(A, 1)];
    output Real p[div(size(A, 1) * (size(A, 1) + 1), 2)];
  algorithm
    p := zeros(div(size(A, 1) * (size(A, 1) + 1), 2));
    for i in 1:size(A, 1) loop
      for j in i:size(A, 1) loop
        p[index(i, j, size(A, 1))] := A[i, j];
      end for;
    end for;
  end pack;

  function unpack "Expand packed upper storage with structural zeros below the diagonal"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real A[n, n];
  algorithm
    A := zeros(n, n);
    for i in 1:n loop
      for j in i:n loop
        A[i, j] := p[index(i, j, n)];
      end for;
    end for;
  end unpack;

  function transposePacked "Transpose packed upper storage into packed lower storage"
    input Integer n;
    input Real p[div(n * (n + 1), 2)];
    output Real q[div(n * (n + 1), 2)];
  algorithm
    q := zeros(div(n * (n + 1), 2));
    for i in 1:n loop
      for j in 1:i loop
        q[LinearAlgebra.LowerTriangular.index(i, j)] := p[index(j, i, n)];
      end for;
    end for;
  end transposePacked;

  function solve "Back substitution on packed storage, with explicit pivot acceptance"
    input Real p[:];
    input Real B[:, :];
    input Boolean positiveDiagonal = false;
    input Real finiteMagnitudeLimit = 1.0e30;
    output Real X[size(B, 1), size(B, 2)];
    output Boolean ok;
  protected
    Integer i;
    Real value;
    Real pivot;
    Boolean usable;
  algorithm
    assert(size(p, 1) == packedSize(size(B, 1)), "UpperTriangular: packed length does not match n");
    X := zeros(size(B, 1), size(B, 2));
    ok := true;
    for reverseRow in 1:size(B, 1) loop
      i := size(B, 1) + 1 - reverseRow;
      pivot := p[index(i, i, size(B, 1))];
      usable := (if positiveDiagonal then pivot > 0.0 else abs(pivot) > 0.0)
        and abs(pivot) < finiteMagnitudeLimit;
      ok := ok and usable;
      for j in 1:size(B, 2) loop
        value := B[i, j];
        for k in i + 1:size(B, 1) loop
          value := value - p[index(i, k, size(B, 1))] * X[k, j];
        end for;
        X[i, j] := if usable then value / pivot else 0.0;
      end for;
    end for;
  end solve;
  annotation(Documentation(info="<html>
<p>Upper factors store {u11, u12, ..., u1n, u22, ..., unn} in row-major
order with n(n+1)/2 entries; index(i,j,n) requires 1 &lt;= i &lt;= j &lt;= n.
pack/unpack preserve structural zeros below the diagonal. transposePacked
converts to LowerTriangular storage.</p>
<p>solve performs backward substitution for multiple right-hand sides and
returns an affirmative pivot-acceptance Boolean. The default magnitude limit
is 1e30; callers can explicitly select their existing numerical domain.</p>
</html>"));
end UpperTriangular;
