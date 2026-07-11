within LieGroups.SO3.EulerB321;
function adjoint "SO(3) adjoint in B321 coordinates"
  input Real element[3];
  output Real Ad[3, 3];
algorithm
  Ad := LieGroups.SO3.EulerB321.to_DCM(element);
end adjoint;
