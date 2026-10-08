within LinearAlgebra;

function covarianceRoot "Lower triangular covariance root from factor columns"
  input Real columns[:, :];
  output Real root[size(columns, 1), size(columns, 1)];
protected
  Real work[max(size(columns, 1), size(columns, 2)), size(columns, 1)];
  Real upperRow[size(columns, 1)];
  Real scale;
  Real radius;
  Real cosine;
  Real sine;
algorithm
  work := cat(1, transpose(columns),
    zeros(max(size(columns, 1) - size(columns, 2), 0), size(columns, 1)));
  for column in 1:size(columns, 1) loop
    for row in 1:size(work, 1) loop
      scale := if row > column then
        max(abs(work[column, column]), abs(work[row, column])) else 0.0;
      if scale > 0.0 then
        radius := sqrt((work[column, column] / scale)^2
          + (work[row, column] / scale)^2);
        cosine := (work[column, column] / scale) / radius;
        sine := (work[row, column] / scale) / radius;
        upperRow := {work[column, entry] for entry in 1:size(columns, 1)};
        work[column, column] := scale * radius;
        work[row, column] := 0.0;
      end if;
      for trailingColumn in 1:size(columns, 1) loop
        if trailingColumn > column and scale > 0.0 then
          work[column, trailingColumn] := cosine * upperRow[trailingColumn]
            + sine * work[row, trailingColumn];
        end if;
      end for;
      for trailingColumn in 1:size(columns, 1) loop
        if trailingColumn > column and scale > 0.0 then
          work[row, trailingColumn] := -sine * upperRow[trailingColumn]
            + cosine * work[row, trailingColumn];
        end if;
      end for;
    end for;
  end for;
  for row in 1:size(columns, 1) loop
    root[:, row] := if work[row, row] < 0.0 then -work[row, :]
      else work[row, :];
  end for;
end covarianceRoot;
