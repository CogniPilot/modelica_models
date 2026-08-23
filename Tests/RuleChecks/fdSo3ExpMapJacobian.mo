within Tests.RuleChecks;
function fdSo3ExpMapJacobian
  "Central difference of SO(3) exp_map, right-trivialized output"
  input Real v[3] "Rotation vector";
  input Real step "Central-difference step";
  output Real J[3, 3];
protected
  Real baseInverse[4];
  Real basis[3, 3];
  Real forward[3];
  Real backward[3];
algorithm
  baseInverse := LieGroups.SO3.Quat.inverse(LieGroups.SO3.Quat.exp_map(v));
  basis := identity(3);
  for j in 1:3 loop
    forward := LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
      baseInverse, LieGroups.SO3.Quat.exp_map(v + step * basis[:, j])));
    backward := LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
      baseInverse, LieGroups.SO3.Quat.exp_map(v - step * basis[:, j])));
    J[:, j] := (forward - backward) / (2.0 * step);
  end for;
end fdSo3ExpMapJacobian;
