within LieGroups.SO3.EulerB321;
function left_jacobian "Map spatial angular velocity to B321 Euler rates"
  input Real element[3];
  output Real J[3, 3];
algorithm
  J := LieGroups.SO3.EulerB321.right_jacobian(element)
    * LieGroups.SO3.EulerB321.to_DCM(element);
end left_jacobian;
