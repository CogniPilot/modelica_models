within LinearAlgebra;

function covarianceRoot "Lower triangular covariance root from factor columns"
  input Real columns[:, :];
  output Real root[size(columns, 1), size(columns, 1)];
protected
  Real work[size(columns, 2), size(columns, 1)];
  Real upperRow[size(columns, 1)];
  Real scale;
  Real radius;
  Real cosine;
  Real sine;
algorithm
  work := transpose(columns);
  for column in 1:size(columns, 1) loop
    for row in (column + 1):size(work, 1) loop
      scale := max(abs(work[column, column]), abs(work[row, column]));
      if scale > 0.0 then
        radius := sqrt((work[column, column] / scale)^2
          + (work[row, column] / scale)^2);
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
  for row in 1:size(columns, 1) loop
    root[:, row] := if row > size(work, 1) then zeros(size(columns, 1))
      elseif work[row, row] < 0.0 then -work[row, :] else work[row, :];
  end for;
end covarianceRoot;
