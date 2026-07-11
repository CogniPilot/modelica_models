within LieGroups.SE23.Generic;
function inverse "SE_2(3) group inverse"
  input Element element;
  output Element result;
protected
  Real inverseR[3, 3];
algorithm
  result.rotation := Rotation.inverse(element.rotation);
  inverseR := Rotation.to_Matrix(result.rotation);
  result.position := -inverseR * element.position;
  result.velocity := -inverseR * element.velocity;
end inverse;
