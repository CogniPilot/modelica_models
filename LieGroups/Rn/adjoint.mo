within LieGroups.Rn;
function adjoint "R^n group adjoint"
  input Real element[:];
  output Real Ad[size(element, 1), size(element, 1)];
algorithm
  Ad := identity(size(element, 1));
end adjoint;
