within MathUtilities;
function norm2 "Euclidean norm of a 2-vector"
  input Real value[2];
  output Real result;
algorithm
  result := sqrt(value * value);
  annotation(Inline = true);
end norm2;
