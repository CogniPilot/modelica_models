within LieGroups.SE3.Generic;
function identity "SE(3) identity"
  output Element element;
algorithm
  element.position := zeros(3);
  element.rotation := Rotation.identity();
end identity;
