within LieGroups.SE23.Quat;
function wedge "se_2(3) tangent to homogeneous algebra matrix"
  input Real tangent[9] "{position tangent, velocity tangent, rotation tangent}";
  output Real matrixRepresentation[5, 5];
algorithm
  matrixRepresentation := zeros(5, 5);
  matrixRepresentation[1:3, 1:3] := LieGroups.SO3.Quat.wedge(tangent[7:9]);
  matrixRepresentation[1:3, 4] := tangent[4:6];
  matrixRepresentation[1:3, 5] := tangent[1:3];
end wedge;
