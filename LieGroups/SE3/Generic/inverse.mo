within LieGroups.SE3.Generic;
function inverse "SE(3) group inverse"
  input Element element;
  output Element result;
algorithm
  result.rotation := Rotation.inverse(element.rotation);
  result.position :=
    -Rotation.to_Matrix(result.rotation) * element.position;
end inverse;
