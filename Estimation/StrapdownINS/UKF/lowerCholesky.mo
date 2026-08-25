within Estimation.StrapdownINS.UKF;

function lowerCholesky
  "Fixed-size lower Cholesky factor with an affirmative success result"
  input Real covariance[TangentLength, TangentLength];
  output Real lower[TangentLength, TangentLength];
  output Boolean success;
protected
  Real pivot;
algorithm
  lower := zeros(TangentLength, TangentLength);
  success := true;
  for row in 1:TangentLength loop
    for column in 1:row loop
      pivot := covariance[row, column];
      for term in 1:(column - 1) loop
        pivot := pivot - lower[row, term] * lower[column, term];
      end for;
      if row == column then
        success := success and pivot > 1.0e-14
          and pivot < Estimation.StrapdownINS.ESKF.FiniteMagnitudeLimit;
        lower[row, column] := sqrt(max(pivot, 1.0e-14));
      else
        lower[row, column] := pivot / lower[column, column];
      end if;
    end for;
  end for;
end lowerCholesky;
