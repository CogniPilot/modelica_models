within LieGroups.SE3.Quat;
function vee "Homogeneous se(3) matrix to tangent vector"
  input Real matrixRepresentation[4, 4];
  output Real tangent[6];
algorithm
  tangent[1:3] := matrixRepresentation[1:3, 4];
  tangent[4:6] := LieGroups.SO3.Quat.vee(matrixRepresentation[1:3, 1:3]);
end vee;
