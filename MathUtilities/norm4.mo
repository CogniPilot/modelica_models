within MathUtilities;
function norm4 "Euclidean norm of a 4-vector"
  input Real value[4];
  output Real result;
algorithm
  result := sqrt(value * value);
  annotation(Inline=true);
end norm4;
