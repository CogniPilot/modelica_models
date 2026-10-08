within LinearAlgebra;

function factorPSD "Lower factor of a positive semidefinite covariance"
  input Real covariance[:, size(covariance, 1)];
  output Real factor[size(covariance, 1), size(covariance, 1)];
  output Boolean ok;
protected
  Real value;
algorithm
  factor := zeros(size(covariance, 1), size(covariance, 1));
  ok := true;
  for row in 1:size(covariance, 1) loop
    for column in 1:row loop
      value := 0.5 * (covariance[row, column] + covariance[column, row]);
      for term in 1:(column - 1) loop
        value := value - factor[row, term] * factor[column, term];
      end for;
      if row == column then
        if value >= 0.0 and abs(value) < 1.0e30 then
          factor[row, column] := sqrt(value);
        else
          ok := false;
        end if;
      elseif factor[column, column] > 0.0 then
        factor[row, column] := value / factor[column, column];
      elseif not abs(value) <= 0.0 then
        ok := false;
      end if;
    end for;
  end for;
end factorPSD;
