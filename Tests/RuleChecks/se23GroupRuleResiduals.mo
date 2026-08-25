within Tests.RuleChecks;
function se23GroupRuleResiduals
  "Worst |rule - central difference| for the SE_2(3) group rules"
  input Real positionScale "Metres of position the sampled elements carry";
  input Real step "Central-difference step";
  input Integer trials "Number of randomized points";
  output Real worst[3] "product left factor, product right factor, inverse";
protected
  Real draw[18];
  Real X1[10];
  Real X2[10];
  // Scalar accumulators: see the note in so3RuleResiduals.
  Real worstProductLeft;
  Real worstProductRight;
  Real worstInverse;
algorithm
  worstProductLeft := 0.0;
  worstProductRight := 0.0;
  worstInverse := 0.0;
  for trial in 1:trials loop
    draw := Tests.RuleChecks.pseudoRandom(104729 * trial + 7, 18);
    X1[1:3] := positionScale * draw[1:3];
    X1[4:6] := 0.2 * positionScale * draw[4:6];
    X1[7:10] := LieGroups.SO3.Quat.exp_map(2.0 * draw[7:9]);
    X2[1:3] := positionScale * draw[10:12];
    X2[4:6] := 0.2 * positionScale * draw[13:15];
    X2[7:10] := LieGroups.SO3.Quat.exp_map(2.0 * draw[16:18]);

    worstProductLeft := max(worstProductLeft, Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.Quat.product_left_factor_jacobian(X1, X2)
      - Tests.RuleChecks.fdSe23ProductJacobian(X1, X2, 1, step)));
    worstProductRight := max(worstProductRight, Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.Quat.product_right_factor_jacobian(X1, X2)
      - Tests.RuleChecks.fdSe23ProductJacobian(X1, X2, 2, step)));
    worstInverse := max(worstInverse, Tests.Assertions.maxAbsMatrix(
      LieGroups.SE23.Quat.inverse_jacobian(X1)
      - Tests.RuleChecks.fdSe23InverseJacobian(X1, step)));
  end for;
  worst := {worstProductLeft, worstProductRight, worstInverse};
end se23GroupRuleResiduals;
