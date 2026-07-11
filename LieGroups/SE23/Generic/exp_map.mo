within LieGroups.SE23.Generic;
function exp_map "SE_2(3) exponential map"
  input Real tangent[9] "{position tangent, velocity tangent, rotation tangent}";
  output Element element;
protected
  Real J[3, 3];
algorithm
  J := LieGroups.SO3.Quat.left_jacobian(tangent[7:9]);
  element.position := J * tangent[1:3];
  element.velocity := J * tangent[4:6];
  element.rotation := Rotation.exp_map(tangent[7:9]);
end exp_map;
