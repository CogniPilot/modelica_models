within LinearAlgebra;
function solve "Dimension-generic dense linear solve with partial pivoting"
  input Real A[:, size(A, 1)];
  input Real B[size(A, 1), :];
  input Real pivotTolerance(min=0.0) = 1.0e-12;
  output Real X[size(A, 1), size(B, 2)];
  output Boolean accepted;
protected
  Integer n = size(A, 1);
  Integer rightHandSides = size(B, 2);
  Integer pivotRow;
  Real factor;
  Real pivotMagnitude;
  Real matrix[size(A, 1), size(A, 1)];
  Real rightHandSide[size(A, 1), size(B, 2)];
  Real temporaryMatrixRow[size(A, 1)];
  Real temporaryRightHandSideRow[size(B, 2)];
algorithm
  assert(n > 0, "Linear solve requires a nonempty square matrix");
  matrix := A;
  rightHandSide := B;
  X := zeros(n, rightHandSides);
  accepted := true;

  for column in 1:n loop
    pivotRow := column;
    pivotMagnitude := abs(matrix[column, column]);
    for row in column + 1:n loop
      if abs(matrix[row, column]) > pivotMagnitude then
        pivotRow := row;
        pivotMagnitude := abs(matrix[row, column]);
      end if;
    end for;
    if pivotMagnitude <= pivotTolerance then
      accepted := false;
    elseif accepted then
      if pivotRow <> column then
        temporaryMatrixRow := matrix[column, :];
        matrix[column, :] := matrix[pivotRow, :];
        matrix[pivotRow, :] := temporaryMatrixRow;
        temporaryRightHandSideRow := rightHandSide[column, :];
        rightHandSide[column, :] := rightHandSide[pivotRow, :];
        rightHandSide[pivotRow, :] := temporaryRightHandSideRow;
      end if;
      for row in column + 1:n loop
        factor := matrix[row, column] / matrix[column, column];
        matrix[row, column:n] := matrix[row, column:n]
          - factor * matrix[column, column:n];
        rightHandSide[row, :] := rightHandSide[row, :]
          - factor * rightHandSide[column, :];
      end for;
    end if;
  end for;

  if accepted then
    for reverseIndex in 1:n loop
      pivotRow := n - reverseIndex + 1;
      for rhsIndex in 1:rightHandSides loop
        X[pivotRow, rhsIndex] := (rightHandSide[pivotRow, rhsIndex]
          - sum(matrix[pivotRow, column] * X[column, rhsIndex]
            for column in pivotRow + 1:n)) / matrix[pivotRow, pivotRow];
      end for;
    end for;
  end if;
end solve;
