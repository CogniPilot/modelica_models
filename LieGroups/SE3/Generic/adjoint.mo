within LieGroups.SE3.Generic;
function adjoint "SE(3) group adjoint"
  input Element element;
  output Real Ad[6, 6];
protected
  Real R[3, 3];
algorithm
  R := Rotation.to_Matrix(element.rotation);
  Ad := zeros(6, 6);
  Ad[1:3, 1:3] := R;
  Ad[1:3, 4:6] := LieGroups.SO3.Quat.wedge(element.position) * R;
  Ad[4:6, 4:6] := R;
end adjoint;
