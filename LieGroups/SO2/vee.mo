within LieGroups.SO2;
function vee "Skew-symmetric matrix to so(2) scalar"
  input Real matrixRepresentation[2, 2];
  output Real omega;
algorithm
  omega := 0.5 * (matrixRepresentation[2, 1] - matrixRepresentation[1, 2]);
end vee;
