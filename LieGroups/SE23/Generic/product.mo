within LieGroups.SE23.Generic;
function product "SE_2(3) group product"
  input Element left;
  input Element right;
  output Element result;
protected
  Real R[3, 3];
algorithm
  R := Rotation.to_Matrix(left.rotation);
  result.position := left.position + R * right.position;
  result.velocity := left.velocity + R * right.velocity;
  result.rotation := Rotation.product(
    left.rotation, right.rotation);
end product;
