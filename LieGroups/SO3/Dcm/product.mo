within LieGroups.SO3.Dcm;
function product "SO(3) DCM product: R1 * R2"
  input Real R1[3,3];
  input Real R2[3,3];
  output Real R[3,3];
algorithm
  R := R1 * R2;
end product;
