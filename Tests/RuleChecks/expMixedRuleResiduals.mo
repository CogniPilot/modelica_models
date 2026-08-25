within Tests.RuleChecks;
function expMixedRuleResiduals
  "Worst |rule - central difference| for the exp_mixed rules"
  input Real angleScale "Magnitude of the increment rotation vectors";
  input Real positionScale "Metres of position the sampled state carries";
  input Real coupling "The dt entry of the nilpotent coupling block";
  input Real step "Central-difference step";
  input Integer trials "Number of randomized points";
  output Real worst[4]
    "initial state, left increment, right increment, increment block against exp_mixed";
protected
  Real draw[24];
  Real X0[10];
  Real l[9];
  Real r[9];
  Real B[2, 2];
  Real axis[3];
  Real axisNorm;
  Real identityElement[10];
  Real reduced[10];
  Real N[3, 2];
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
  identityElement := {0, 0, 0, 0, 0, 0, 1, 0, 0, 0};
  B := [0.0, coupling; 0.0, 0.0];
  for trial in 1:trials loop
    draw := Tests.RuleChecks.pseudoRandom(48611 * trial + 101, 24);
    X0[1:3] := positionScale * draw[1:3];
    X0[4:6] := 0.2 * positionScale * draw[4:6];
    X0[7:10] := LieGroups.SO3.Quat.exp_map(2.0 * draw[7:9]);

    l[1:3] := 0.1 * draw[10:12];
    l[4:6] := draw[13:15];
    axis := draw[16:18];
    axisNorm := sqrt(axis[1]^2 + axis[2]^2 + axis[3]^2);
    l[7:9] := (angleScale / max(axisNorm, 1.0e-12)) * axis;

    r[1:3] := 0.1 * draw[19:21];
    r[4:6] := draw[22:24];
    axis := {draw[1], draw[13], draw[7]};
    axisNorm := sqrt(axis[1]^2 + axis[2]^2 + axis[3]^2);
    r[7:9] := (angleScale / max(axisNorm, 1.0e-12)) * axis;

    worstState := max(worstState, Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.Quat.exp_mixed_state_jacobian(X0, l, r, B)
      - Tests.RuleChecks.fdExpMixedJacobian(X0, l, r, B, 1, step)));
    worstLeft := max(worstLeft, Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.Quat.exp_mixed_left_increment_jacobian(X0, l, r, B)
      - Tests.RuleChecks.fdExpMixedJacobian(X0, l, r, B, 2, step)));
    worstRight := max(worstRight, Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.Quat.exp_mixed_right_increment_jacobian(X0, l, r, B)
      - Tests.RuleChecks.fdExpMixedJacobian(X0, l, r, B, 3, step)));

    // exp_mixed at the identity with a zero right increment reduces to the
    // increment block itself, which pins the factored block against the
    // primitive the rules are supposed to differentiate.
    reduced := LieGroups.SE23.Quat.exp_mixed(identityElement, l, zeros(9), B);
    N := LieGroups.SE23.Quat.mixed_increment_matrix(l, B);
    worstBlock := max(worstBlock, max(
      Tests.Assertions.maxAbsVector(reduced[1:3] - N[:, 2]),
      Tests.Assertions.maxAbsVector(reduced[4:6] - N[:, 1])));
  end for;
  worst := {worstState, worstLeft, worstRight, worstBlock};
end expMixedRuleResiduals;
