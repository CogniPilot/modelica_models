within MathUtilities;
function lowPass3 "Apply a first-order sample blend to a 3-vector"
  input Real sample[3];
  input Real previous[3];
  input Real sampleWeight;
  output Real result[3];
algorithm
  result := sampleWeight * sample + (1.0 - sampleWeight) * previous;
  annotation(Inline = true);
end lowPass3;
