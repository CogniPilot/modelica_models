within LieGroups.SE3.Generic;
function log_map "SE(3) logarithmic map"
  input Element element;
  output Real tangent[6] "{translation, rotation}";
algorithm
  tangent[4:6] := Rotation.log_map(element.rotation);
  tangent[1:3] := LieGroups.SO3.Quat.left_jacobian_inv(tangent[4:6])
    * element.position;
end log_map;
