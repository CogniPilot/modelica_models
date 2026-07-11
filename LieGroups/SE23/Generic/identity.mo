within LieGroups.SE23.Generic;
function identity "SE_2(3) identity"
  output Element element;
algorithm
  element.position := zeros(3);
  element.velocity := zeros(3);
  element.rotation := Rotation.identity();
end identity;
