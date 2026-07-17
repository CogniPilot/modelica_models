within MathUtilities;
function lowPass "Apply a scalar first-order sample blend"
  input Real sample;
  input Real previous;
  input Real sampleWeight;
  output Real result;
algorithm
  result := sampleWeight * sample + (1.0 - sampleWeight) * previous;
  annotation(Inline = true);
end lowPass;
