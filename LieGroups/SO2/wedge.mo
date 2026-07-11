within LieGroups.SO2;
function wedge "so(2) scalar to skew-symmetric matrix"
  input Real omega;
  output Real matrixRepresentation[2, 2];
algorithm
  matrixRepresentation := [0.0, -omega; omega, 0.0];
end wedge;
