within Tests.Assertions;
function maxAbsVector "Maximum absolute element of a vector"
  input Real values[:];
  output Real result;
algorithm
  result := 0.0;
  for i in 1:size(values, 1) loop
    result := max(result, abs(values[i]));
  end for;
end maxAbsVector;
