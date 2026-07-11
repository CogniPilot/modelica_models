within LieGroups.SE3.Generic;
function product "SE(3) group product"
  input Element left;
  input Element right;
  output Element result;
algorithm
  result.position := left.position
    + Rotation.to_Matrix(left.rotation) * right.position;
  result.rotation := Rotation.product(
    left.rotation, right.rotation);
end product;
