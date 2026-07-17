within MathUtilities;
function norm3 "Euclidean norm of a 3-vector"
  input Real value[3];
  output Real result;
algorithm
  result := sqrt(value * value);
  annotation(Inline = true);
end norm3;
