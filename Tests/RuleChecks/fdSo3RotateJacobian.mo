within Tests.RuleChecks;
function fdSo3RotateJacobian
  "Central difference of SO(3) rotate in one of its two slots"
  input Real q[4] "Unit quaternion";
  input Real v[3] "Vector being rotated";
  input Integer slot "1 for the rotation, 2 for the vector";
  input Real step "Central-difference step";
  output Real J[3, 3];
protected
  Real basis[3, 3];
  Real forward[3];
  Real backward[3];
algorithm
  basis := identity(3);
  for j in 1:3 loop
    if slot == 1 then
      forward := LieGroups.SO3.Quat.rotate(LieGroups.SO3.Quat.product(
        q, LieGroups.SO3.Quat.exp_map(step * basis[:, j])), v);
      backward := LieGroups.SO3.Quat.rotate(LieGroups.SO3.Quat.product(
        q, LieGroups.SO3.Quat.exp_map(-step * basis[:, j])), v);
    else
      forward := LieGroups.SO3.Quat.rotate(q, v + step * basis[:, j]);
      backward := LieGroups.SO3.Quat.rotate(q, v - step * basis[:, j]);
    end if;
    J[:, j] := (forward - backward) / (2.0 * step);
  end for;
end fdSo3RotateJacobian;
