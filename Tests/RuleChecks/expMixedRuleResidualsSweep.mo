within Tests.RuleChecks;
function expMixedRuleResidualsSweep
  "Worst |rule - central difference| for the exp_mixed rules over a magnitude list"
  input Real angleScales[:] "Increment rotation magnitudes to sweep";
  input Real positionScale "Metres of position the sampled state carries";
  input Real coupling "The dt entry of the nilpotent coupling block";
  input Real step "Central-difference step";
  input Integer trials "Randomized points per magnitude";
  output Real worst[4]
    "The four entries of expMixedRuleResiduals, maximised over the sweep";
protected
  Real row[4];
  // Scalar accumulators: see the note in so3RuleResiduals.
  Real worstState;
  Real worstLeft;
  Real worstRight;
  Real worstBlock;
algorithm
  worstState := 0.0;
  worstLeft := 0.0;
  worstRight := 0.0;
  worstBlock := 0.0;
  for i in 1:size(angleScales, 1) loop
    row := Tests.RuleChecks.expMixedRuleResiduals(
      angleScales[i], positionScale, coupling, step, trials);
    worstState := max(worstState, row[1]);
    worstLeft := max(worstLeft, row[2]);
    worstRight := max(worstRight, row[3]);
    worstBlock := max(worstBlock, row[4]);
  end for;
  worst := {worstState, worstLeft, worstRight, worstBlock};
  annotation(Documentation(info="<html>
    <p>A per-rule maximum over a magnitude list. The position scale stays a
    single argument because the three exp_mixed rules carry it as a lever arm:
    the difference floor, and the amount by which any error in the SO(3) right
    Jacobian is amplified, are both proportional to it, so a sweep is taken at
    one lever arm at a time and the budget is stated against that arm.</p>
  </html>"));
end expMixedRuleResidualsSweep;
