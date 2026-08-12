within LinearAlgebra;
function solveSPD
  "Solve A*X=B for symmetric positive-definite A using Cholesky"
  input Real A[:, size(A, 1)] "Symmetric positive-definite system matrix";
  input Real B[size(A, 1), :] "One or more right-hand sides";
  input Real relativeTolerance = 0.0
    "Optional extra floor on the Cholesky pivot threshold relative to the
     largest input diagonal; the working-precision term always applies";
  output Real X[size(B, 1), size(B, 2)] "Solution";
  output Boolean ok "False when A is not numerically positive definite";
protected
  Real L[size(A, 1), size(A, 2)] "Lower Cholesky factor";
  Real Y[size(B, 1), size(B, 2)] "Forward-substitution result";
  Real value;
  Real scale;
  Real one;
  Real workingEpsilon;
  Real pivotThreshold;
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

  // Measure the machine epsilon of the precision actually executing this
  // function. The same source is evaluated as binary64 during simulation
  // and as binary32 in generated GALEC flight code, so no fixed literal
  // can serve both: binary32 epsilon is 2^-23 ~ 1.19e-7, and any fixed
  // relative tolerance far below it (the former default was 1.0e-12)
  // is unreachable in binary32 - the pivot check degenerates to
  // "value <= ~0", so pivots destroyed by rounding are accepted until one
  // finally goes non-positive, after which the factorization is rejected
  // on every call. Halving from 1.0 until adding half of it no longer
  // changes 1.0 converges to the epsilon of the working precision; "one"
  // is derived from the input data so a constant-folding pass evaluated
  // in a wider precision cannot bake in the wrong epsilon. 60 guarded
  // iterations cover binary64 (epsilon 2^-52); once converged the guard
  // fails and the remaining iterations are no-ops, negligible next to
  // the O(n^3) factorization.
  one := scale / scale;
  workingEpsilon := 1.0;
  for k in 1:60 loop
    if one + 0.5 * workingEpsilon > one then
      workingEpsilon := 0.5 * workingEpsilon;
    end if;
  end for;

  // Scaled Cholesky pivot criterion: a pivot at or below
  // n * epsilon * max|A[i,i]| is indistinguishable from accumulated
  // rounding noise in the working precision (cf. Higham, "Accuracy and
  // Stability of Numerical Algorithms", ch. 10). This yields ~1.8e-6
  // relative in binary32 (n = 15) and ~3.3e-15 in binary64. Callers can
  // only tighten the threshold via relativeTolerance, never defeat the
  // precision floor.
  pivotThreshold :=
    max(relativeTolerance, size(A, 1) * workingEpsilon) * scale;
  ok := true;

  for row in 1:size(A, 1) loop
    for column in 1:row loop
      value := 0.5 * (A[row, column] + A[column, row]);
      for k in 1:(column - 1) loop
        value := value - L[row, k] * L[column, k];
      end for;

      if row == column then
        if value <= pivotThreshold then
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
        // DO NOT add a multi-output (tuple-assigning) call anywhere in this
        // function. rumoca 0.9.20 silently drops loop-carried writes when
        // three ingredients coincide: (1) a nested loop whose trip count
        // depends on an enclosing loop index, (2) that loop reading an array
        // indexed by its own inner variable, and (3) a multi-output call in
        // the enclosing loop. The back substitution below already has (1) and
        // (2) -- `(solveRow + 1):size(A, 1)` reading `L[k, solveRow]` and
        // `X[k, rhs]`, accumulating into `value` across iterations -- so only
        // (3) is missing. Adding it would silently return a wrong solution
        // from the flight estimator's Cholesky path, with no diagnostic.
        // tools/check-modelica-library.sh enforces the absence of (3); drop
        // that check only once the compiler defect is fixed.
        // Evidence: PlanTrigger2.mo, PlanShapeProbe.mo and PlanJointShape2.mo
        // under .claude/jobs/de80c98d/tmp/planaudit/. The real instance was
        // DubinsPolynomial.smoothOffsets, repaired in commit 2b255b3.
        for k in (solveRow + 1):size(A, 1) loop
          value := value - L[k, solveRow] * X[k, rhs];
        end for;
        X[solveRow, rhs] := value / L[solveRow, solveRow];
      end for;
    end for;
  end if;
end solveSPD;
