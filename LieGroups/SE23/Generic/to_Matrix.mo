within LieGroups.SE23.Generic;
function to_Matrix "SE_2(3) homogeneous matrix"
  input Element element;
  output Real matrixRepresentation[5, 5];
algorithm
  matrixRepresentation := zeros(5, 5);
  for i in 1:5 loop
    matrixRepresentation[i, i] := 1.0;
  end for;
  matrixRepresentation[1:3, 1:3] :=
    Rotation.to_Matrix(element.rotation);
  matrixRepresentation[1:3, 4] := element.velocity;
  matrixRepresentation[1:3, 5] := element.position;
end to_Matrix;
