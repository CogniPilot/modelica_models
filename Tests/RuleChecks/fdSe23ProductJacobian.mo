within Tests.RuleChecks;
function fdSe23ProductJacobian
  "Central difference of the SE_2(3) product in one of its two slots"
  input Real X1[10] "Left element";
  input Real X2[10] "Right element";
  input Integer slot "1 for the left factor, 2 for the right factor";
  input Real step "Central-difference step";
  output Real J[9, 9];
protected
  Real baseInverse[10];
  Real basis[9, 9];
  Real forwardStep[10];
  Real backwardStep[10];
  Real forwardElement[10];
  Real backwardElement[10];
algorithm
  baseInverse := LieGroups.SE23.Quat.inverse(
    LieGroups.SE23.Quat.product(X1, X2));
  basis := identity(9);
  for j in 1:9 loop
    forwardStep := LieGroups.SE23.Quat.exp_map(step * basis[:, j]);
    backwardStep := LieGroups.SE23.Quat.exp_map(-step * basis[:, j]);
    if slot == 1 then
      forwardElement := LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.product(X1, forwardStep), X2);
      backwardElement := LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.product(X1, backwardStep), X2);
    else
      forwardElement := LieGroups.SE23.Quat.product(
        X1, LieGroups.SE23.Quat.product(X2, forwardStep));
      backwardElement := LieGroups.SE23.Quat.product(
        X1, LieGroups.SE23.Quat.product(X2, backwardStep));
    end if;
    J[:, j] := (LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
        baseInverse, forwardElement))
      - LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
        baseInverse, backwardElement))) / (2.0 * step);
  end for;
end fdSe23ProductJacobian;
