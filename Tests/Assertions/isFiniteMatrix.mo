within Tests.Assertions;
function isFiniteMatrix "True when every matrix entry is finite"
  input Real values[:, :];
  output Boolean result;
algorithm
  result := true;
  for row in 1:size(values, 1) loop
    for column in 1:size(values, 2) loop
      result := result and values[row, column] == values[row, column]
        and abs(values[row, column]) < 1.0e100;
    end for;
  end for;
end isFiniteMatrix;
