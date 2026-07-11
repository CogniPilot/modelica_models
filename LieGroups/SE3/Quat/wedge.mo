within LieGroups.SE3.Quat;
function wedge "se(3) vector to homogeneous algebra matrix"
  input Real tangent[6] "{translation, rotation}";
  output Real matrixRepresentation[4, 4];
algorithm
  matrixRepresentation := zeros(4, 4);
  matrixRepresentation[1:3, 1:3] := LieGroups.SO3.Quat.wedge(tangent[4:6]);
  matrixRepresentation[1:3, 4] := tangent[1:3];
end wedge;
