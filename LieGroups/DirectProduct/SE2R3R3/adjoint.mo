within LieGroups.DirectProduct.SE2R3R3;
function adjoint "Direct-product adjoint"
  input Real element[9];
  output Real Ad[9, 9];
algorithm
  Ad := zeros(9, 9);
  Ad[1:3, 1:3] := LieGroups.SE2.adjoint(element[1:3]);
  Ad[4:6, 4:6] := identity(3);
  Ad[7:9, 7:9] := identity(3);
end adjoint;
