within Tests.Assertions;
function isFiniteVector "True when every vector entry is finite"
  input Real values[:];
  output Boolean result;
algorithm
  result := true;
  for i in 1:size(values, 1) loop
    result := result and values[i] == values[i] and abs(values[i]) < Estimation.MultiSensorInvariant.FiniteMagnitudeLimit;
  end for;
end isFiniteVector;
