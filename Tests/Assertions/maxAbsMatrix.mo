within Tests.Assertions;
function maxAbsMatrix "Maximum absolute element of a matrix"
  input Real values[:, :];
  output Real result;
algorithm
  result := 0.0;
  for row in 1:size(values, 1) loop
    for column in 1:size(values, 2) loop
      result := max(result, abs(values[row, column]));
    end for;
  end for;
end maxAbsMatrix;
