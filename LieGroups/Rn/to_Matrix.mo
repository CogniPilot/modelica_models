within LieGroups.Rn;
function to_Matrix "Homogeneous matrix representation of an R^n translation"
  input Real element[:];
  output Real matrixRepresentation[size(element, 1) + 1,
                                   size(element, 1) + 1];
algorithm
  matrixRepresentation := identity(size(element, 1) + 1);
  matrixRepresentation[1:size(element, 1), size(element, 1) + 1] := element;
end to_Matrix;
