within LieGroups.Rn;
function vee "Recover an R^n algebra vector from its homogeneous matrix"
  input Real matrixRepresentation[:, size(matrixRepresentation, 1)];
  output Real tangent[size(matrixRepresentation, 1) - 1];
algorithm
  tangent := matrixRepresentation[
    1:(size(matrixRepresentation, 1) - 1), size(matrixRepresentation, 1)];
end vee;
