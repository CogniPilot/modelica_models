within LieGroups.SE23.Quat;
function vee "Homogeneous se_2(3) matrix to tangent"
  input Real matrixRepresentation[5, 5];
  output Real tangent[9];
algorithm
  tangent[1:3] := matrixRepresentation[1:3, 5];
  tangent[4:6] := matrixRepresentation[1:3, 4];
  tangent[7:9] := LieGroups.SO3.Quat.vee(matrixRepresentation[1:3, 1:3]);
end vee;
