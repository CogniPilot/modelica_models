within LieGroups.SE23.Generic;
function adjoint "SE_2(3) group adjoint"
  input Element element;
  output Real Ad[9, 9];
protected
  Real R[3, 3];
algorithm
  R := Rotation.to_Matrix(element.rotation);
  Ad := zeros(9, 9);
  Ad[1:3, 1:3] := R;
  Ad[1:3, 7:9] := LieGroups.SO3.Quat.wedge(element.position) * R;
  Ad[4:6, 4:6] := R;
  Ad[4:6, 7:9] := LieGroups.SO3.Quat.wedge(element.velocity) * R;
  Ad[7:9, 7:9] := R;
end adjoint;
