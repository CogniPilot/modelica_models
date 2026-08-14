within MathUtilities;
function clipVector "Clamp every element of a vector to a closed interval"
  input Real value[:];
  input Real lower;
  input Real upper;
  output Real result[size(value, 1)];
algorithm
  for index in 1:size(value, 1) loop
    result[index] := MathUtilities.clip(value[index], lower, upper);
  end for;
  annotation(Inline = true);
end clipVector;
