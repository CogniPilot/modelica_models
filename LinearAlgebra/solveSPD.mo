within LinearAlgebra;
function solveSPD
  "Solve A*X=B for symmetric positive-definite A using Cholesky"
  input Real A[:, size(A, 1)] "Symmetric positive-definite system matrix";
  input Real B[size(A, 1), :] "One or more right-hand sides";
  input Real relativeTolerance = 1.0e-12
    "Minimum Cholesky pivot relative to the largest input diagonal";
  output Real X[size(B, 1), size(B, 2)] "Solution";
  output Boolean ok "False when A is not numerically positive definite";
protected
  Real L[size(A, 1), size(A, 2)] "Lower Cholesky factor";
  Real Y[size(B, 1), size(B, 2)] "Forward-substitution result";
  Real value;
  Real scale;
  Integer solveRow;
algorithm
  L := zeros(size(A, 1), size(A, 2));
  Y := zeros(size(B, 1), size(B, 2));
  X := zeros(size(B, 1), size(B, 2));
  scale := 0.0;
  for i in 1:size(A, 1) loop
    scale := max(scale, abs(A[i, i]));
  end for;
  scale := max(scale, 1.0e-30);
  ok := true;

  for row in 1:size(A, 1) loop
    for column in 1:row loop
      value := 0.5 * (A[row, column] + A[column, row]);
      for k in 1:(column - 1) loop
        value := value - L[row, k] * L[column, k];
      end for;

      if row == column then
        if value <= relativeTolerance * scale then
          ok := false;
          // Keep subsequent arithmetic finite for tools that continue
          // evaluating after a failed pivot; X is rejected when ok=false.
          L[row, column] := 1.0;
        else
          L[row, column] := sqrt(value);
        end if;
      else
        L[row, column] := value / L[column, column];
      end if;
    end for;
  end for;

  if ok then
    for rhs in 1:size(B, 2) loop
      for row in 1:size(A, 1) loop
        value := B[row, rhs];
        for k in 1:(row - 1) loop
          value := value - L[row, k] * Y[k, rhs];
        end for;
        Y[row, rhs] := value / L[row, row];
      end for;

      for reverseRow in 1:size(A, 1) loop
        solveRow := size(A, 1) + 1 - reverseRow;
        value := Y[solveRow, rhs];
        for k in (solveRow + 1):size(A, 1) loop
          value := value - L[k, solveRow] * X[k, rhs];
        end for;
        X[solveRow, rhs] := value / L[solveRow, solveRow];
      end for;
    end for;
  end if;
end solveSPD;
