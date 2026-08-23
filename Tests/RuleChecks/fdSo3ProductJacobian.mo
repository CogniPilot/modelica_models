within Tests.RuleChecks;
function fdSo3ProductJacobian
  "Central difference of the SO(3) product in one of its two slots"
  input Real q[4] "Left quaternion";
  input Real p[4] "Right quaternion";
  input Integer slot "1 for the left factor, 2 for the right factor";
  input Real step "Central-difference step";
  output Real J[3, 3];
protected
  Real baseInverse[4];
  Real basis[3, 3];
  Real forwardStep[4];
  Real backwardStep[4];
  Real forwardElement[4];
  Real backwardElement[4];
algorithm
  baseInverse := LieGroups.SO3.Quat.inverse(
    LieGroups.SO3.Quat.product(q, p));
  basis := identity(3);
  for j in 1:3 loop
    forwardStep := LieGroups.SO3.Quat.exp_map(step * basis[:, j]);
    backwardStep := LieGroups.SO3.Quat.exp_map(-step * basis[:, j]);
    if slot == 1 then
      forwardElement := LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.product(q, forwardStep), p);
      backwardElement := LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.product(q, backwardStep), p);
    else
      forwardElement := LieGroups.SO3.Quat.product(
        q, LieGroups.SO3.Quat.product(p, forwardStep));
      backwardElement := LieGroups.SO3.Quat.product(
        q, LieGroups.SO3.Quat.product(p, backwardStep));
    end if;
    J[:, j] := (LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
        baseInverse, forwardElement))
      - LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
        baseInverse, backwardElement))) / (2.0 * step);
  end for;
end fdSo3ProductJacobian;
