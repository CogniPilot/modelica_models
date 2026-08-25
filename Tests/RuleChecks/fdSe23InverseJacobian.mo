within Tests.RuleChecks;
function fdSe23InverseJacobian
  "Central difference of SE_2(3) inverse, right-trivialized on both sides"
  input Real X[10] "Element";
  input Real step "Central-difference step";
  output Real J[9, 9];
protected
  Real basis[9, 9];
  Real forward[9];
  Real backward[9];
algorithm
  basis := identity(9);
  for j in 1:9 loop
    forward := LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
      X, LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.product(
        X, LieGroups.SE23.Quat.exp_map(step * basis[:, j])))));
    backward := LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.product(
      X, LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.product(
        X, LieGroups.SE23.Quat.exp_map(-step * basis[:, j])))));
    J[:, j] := (forward - backward) / (2.0 * step);
  end for;
end fdSe23InverseJacobian;
