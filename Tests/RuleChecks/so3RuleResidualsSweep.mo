within Tests.RuleChecks;
function so3RuleResidualsSweep
  "Worst |rule - central difference| over a whole list of rotation magnitudes"
  input Real angleScales[:] "Rotation magnitudes to sweep";
  input Real step "Central-difference step";
  input Integer trials "Randomized points per magnitude";
  output Real worst[7]
    "The seven rules of so3RuleResiduals, maximised over the sweep";
protected
  Real row[7];
  // Scalar accumulators rather than accumulation into the output array: see
  // the note in so3RuleResiduals.
  Real worstExponential;
  Real worstLogarithm;
  Real worstProductLeft;
  Real worstProductRight;
  Real worstInverse;
  Real worstRotateRotation;
  Real worstRotateVector;
algorithm
  worstExponential := 0.0;
  worstLogarithm := 0.0;
  worstProductLeft := 0.0;
  worstProductRight := 0.0;
  worstInverse := 0.0;
  worstRotateRotation := 0.0;
  worstRotateVector := 0.0;
  for i in 1:size(angleScales, 1) loop
    row := Tests.RuleChecks.so3RuleResiduals(angleScales[i], step, trials);
    worstExponential := max(worstExponential, row[1]);
    worstLogarithm := max(worstLogarithm, row[2]);
    worstProductLeft := max(worstProductLeft, row[3]);
    worstProductRight := max(worstProductRight, row[4]);
    worstInverse := max(worstInverse, row[5]);
    worstRotateRotation := max(worstRotateRotation, row[6]);
    worstRotateVector := max(worstRotateVector, row[7]);
  end for;
  worst := {worstExponential, worstLogarithm, worstProductLeft,
    worstProductRight, worstInverse, worstRotateRotation, worstRotateVector};
  annotation(Documentation(info="<html>
    <p>A per-rule maximum over a magnitude list, so that a budget can be stated
    once per rule instead of once per rule and magnitude. The list is the
    caller's, which is what lets one model sweep the whole range densely while
    still naming individual magnitudes it wants reported separately.</p>
  </html>"));
end so3RuleResidualsSweep;
