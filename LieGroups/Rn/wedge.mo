within LieGroups.Rn;
function wedge "Homogeneous matrix representation of an R^n algebra element"
  input Real tangent[:];
  output Real matrixRepresentation[size(tangent, 1) + 1,
                                   size(tangent, 1) + 1];
algorithm
  matrixRepresentation := zeros(size(tangent, 1) + 1, size(tangent, 1) + 1);
  matrixRepresentation[1:size(tangent, 1), size(tangent, 1) + 1] := tangent;
end wedge;
