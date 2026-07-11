within LieGroups.SE23.Generic;
function log_map "SE_2(3) logarithmic map"
  input Element element;
  output Real tangent[9];
protected
  Real inverseJ[3, 3];
algorithm
  tangent[7:9] := Rotation.log_map(element.rotation);
  inverseJ := LieGroups.SO3.Quat.left_jacobian_inv(tangent[7:9]);
  tangent[1:3] := inverseJ * element.position;
  tangent[4:6] := inverseJ * element.velocity;
end log_map;
