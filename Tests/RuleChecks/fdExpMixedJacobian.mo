within Tests.RuleChecks;
function fdExpMixedJacobian
  "Central difference of exp_mixed in one of its three differentiated slots"
  input Real X0[10] "Initial state";
  input Real l[9] "Left increment";
  input Real r[9] "Right increment";
  input Real B[2, 2] "Coupling matrix";
  input Integer slot "1 initial state, 2 left increment, 3 right increment";
  input Real step "Central-difference step";
  output Real J[9, 9];
protected
  Real baseInverse[10];
  Real basis[9, 9];
  Real forwardElement[10];
  Real backwardElement[10];
algorithm
  baseInverse := LieGroups.SE23.Quat.inverse(
    LieGroups.SE23.Quat.exp_mixed(X0, l, r, B));
  basis := identity(9);
  for j in 1:9 loop
    if slot == 1 then
      forwardElement := LieGroups.SE23.Quat.exp_mixed(
        LieGroups.SE23.Quat.product(X0,
          LieGroups.SE23.Quat.exp_map(step * basis[:, j])), l, r, B);
      backwardElement := LieGroups.SE23.Quat.exp_mixed(
        LieGroups.SE23.Quat.product(X0,
          LieGroups.SE23.Quat.exp_map(-step * basis[:, j])), l, r, B);
    elseif slot == 2 then
      forwardElement := LieGroups.SE23.Quat.exp_mixed(
        X0, l + step * basis[:, j], r, B);
      backwardElement := LieGroups.SE23.Quat.exp_mixed(
        X0, l - step * basis[:, j], r, B);
    else
      forwardElement := LieGroups.SE23.Quat.exp_mixed(
        X0, l, r + step * basis[:, j], B);
      backwardElement := LieGroups.SE23.Quat.exp_mixed(
        X0, l, r - step * basis[:, j], B);
    end if;
    J[:, j] := (LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
        baseInverse, forwardElement))
      - LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
        baseInverse, backwardElement))) / (2.0 * step);
  end for;
end fdExpMixedJacobian;
