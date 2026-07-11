within LieGroups.SO3.Dcm;
function inverse "SO(3) DCM inverse: transpose"
  input Real R[3,3];
  output Real R_inv[3,3];
algorithm
  R_inv := transpose(R);
end inverse;
