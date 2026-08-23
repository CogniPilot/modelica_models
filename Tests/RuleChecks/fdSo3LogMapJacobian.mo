within Tests.RuleChecks;
function fdSo3LogMapJacobian
  "Central difference of SO(3) log_map, right-trivialized input"
  input Real q[4] "Unit quaternion";
  input Real step "Central-difference step";
  output Real J[3, 3];
protected
  Real basis[3, 3];
  Real forward[3];
  Real backward[3];
algorithm
  basis := identity(3);
  for j in 1:3 loop
    forward := LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
      q, LieGroups.SO3.Quat.exp_map(step * basis[:, j])));
    backward := LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
      q, LieGroups.SO3.Quat.exp_map(-step * basis[:, j])));
    J[:, j] := (forward - backward) / (2.0 * step);
  end for;
end fdSo3LogMapJacobian;
