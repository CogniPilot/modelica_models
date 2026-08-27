within Estimation.StrapdownINS.UKF;

function lowerCholesky
  "Fixed-size lower Cholesky factor with an affirmative success result"
  input Real covariance[TangentLength, TangentLength];
  output Real lower[TangentLength, TangentLength];
  output Boolean success;
protected
  Real pivot;
  Real scale;
  Real one;
  Real workingEpsilon;
  Real pivotThreshold;
algorithm
  lower := zeros(TangentLength, TangentLength);

  // Largest diagonal entry sets the scale the pivot criterion is relative to.
  // A covariance is positive semi-definite, so this is also the largest entry
  // of the whole matrix.
  scale := 0.0;
  for i in 1:TangentLength loop
    scale := max(scale, abs(covariance[i, i]));
  end for;
  scale := max(scale, 1.0e-30);

  // Measure the machine epsilon of the precision actually executing this
  // function, exactly as LinearAlgebra.solveSPD does and for the same reason:
  // the same source runs as binary64 in simulation and as binary32 in
  // generated flight code, so no fixed literal serves both. The former
  // absolute 1.0e-14 floor is roughly 1.0e-8 relative for the covariance
  // magnitudes this filter carries, which is below binary32 epsilon
  // (2^-23 ~ 1.19e-7): the test degenerated to "pivot <= ~0", so pivots
  // already destroyed by rounding were accepted, and the sigma points were
  // built from a factor that does not reproduce the covariance.
  one := scale / scale;
  workingEpsilon := 1.0;
  for k in 1:60 loop
    if one + 0.5 * workingEpsilon > one then
      workingEpsilon := 0.5 * workingEpsilon;
    end if;
  end for;

  // Scaled Cholesky pivot criterion: a pivot at or below
  // n * epsilon * max|A[i,i]| is indistinguishable from accumulated rounding
  // noise in the working precision (Higham, "Accuracy and Stability of
  // Numerical Algorithms", ch. 10). About 1.8e-6 relative in binary32 and
  // 3.3e-15 in binary64 at n = 15.
  pivotThreshold := TangentLength * workingEpsilon * scale;

  success := true;
  for row in 1:TangentLength loop
    for column in 1:row loop
      pivot := covariance[row, column];
      for term in 1:(column - 1) loop
        pivot := pivot - lower[row, term] * lower[column, term];
      end for;
      if row == column then
        success := success and pivot > pivotThreshold
          and pivot < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
        // The floor keeps the remaining divisions finite on a rejected
        // factorization. It is the same quantity the acceptance test uses, so
        // a factor that was floored is exactly the one the caller is told not
        // to trust, rather than a silently plausible one.
        lower[row, column] := sqrt(max(pivot, pivotThreshold));
      else
        lower[row, column] := pivot / lower[column, column];
      end if;
    end for;
  end for;
end lowerCholesky;
