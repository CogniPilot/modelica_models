within LieGroups.SE3.Generic;
function to_Matrix "SE(3) homogeneous matrix"
  input Element element;
  output Real matrixRepresentation[4, 4];
algorithm
  matrixRepresentation := zeros(4, 4);
  for i in 1:4 loop
    matrixRepresentation[i, i] := 1.0;
  end for;
  matrixRepresentation[1:3, 1:3] :=
    Rotation.to_Matrix(element.rotation);
  matrixRepresentation[1:3, 4] := element.position;
end to_Matrix;
