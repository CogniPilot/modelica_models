within LieGroups.SE3.Generic;
function exp_map "SE(3) exponential map"
  input Real tangent[6] "{translation, rotation}";
  output Element element;
algorithm
  element.position := LieGroups.SO3.Quat.left_jacobian(tangent[4:6])
    * tangent[1:3];
  element.rotation := Rotation.exp_map(tangent[4:6]);
end exp_map;
