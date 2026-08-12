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
algorithm
  assert(n > 0, "Linear solve requires a nonempty square matrix");
  matrix := A;
  rightHandSide := B;
  X := zeros(n, rightHandSides);
  accepted := true;

  for column in 1:n loop
    pivotRow := column;
    pivotMagnitude := abs(matrix[column, column]);
    // DO NOT add a multi-output (tuple-assigning) call anywhere in this
    // function. rumoca 0.9.20 silently drops loop-carried writes when three
    // ingredients coincide: (1) a nested loop whose trip count depends on the
    // enclosing loop index, (2) that loop reading an array indexed by its own
    // inner variable, and (3) a multi-output call in the enclosing loop. The
    // pivot search below already has (1) and (2) -- `column + 1:n` reading
    // `matrix[row, column]`, with pivotRow and pivotMagnitude carried across
    // iterations -- so only (3) is missing. Adding it would make this loop
    // select the wrong pivot with no diagnostic, silently corrupting every
    // solve that reaches here, including the flight estimator's covariance
    // updates. tools/check-modelica-library.sh enforces the absence of (3);
    // drop that check only once the compiler defect is fixed.
    // Evidence: PlanTrigger2.mo isolates the three ingredients, PlanShapeProbe.mo
    // shows (2) absent is correct, PlanJointShape2.mo shows (1) absent is
    // correct; all under .claude/jobs/de80c98d/tmp/planaudit/. The real
    // instance was DubinsPolynomial.smoothOffsets, repaired in commit 2b255b3.
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
        matrix[{column, pivotRow}, :] := matrix[{pivotRow, column}, :];
        rightHandSide[{column, pivotRow}, :] :=
          rightHandSide[{pivotRow, column}, :];
      end if;
      // Same hazard as the pivot search above: `column + 1:n` reading
      // `matrix[row, column]`, mutating `matrix` and `rightHandSide` across
      // iterations. A multi-output call added to this function would corrupt
      // the elimination itself, not just pivot selection. See the note above.
      for row in column + 1:n loop
        factor := matrix[row, column] / matrix[column, column];
        rightHandSide[row, :] := rightHandSide[row, :]
          - factor * rightHandSide[column, :];
        matrix[row, column:n] := matrix[row, column:n]
          - factor * matrix[column, column:n];
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
