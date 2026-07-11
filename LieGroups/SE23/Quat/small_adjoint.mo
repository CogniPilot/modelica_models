within LieGroups.SE23.Quat;
function small_adjoint "Small adjoint ad_xi for se_2(3)"
  input Real xi[9] "Lie algebra {vb, ab, omega}";
  output Real ad[9,9] "9x9 small adjoint matrix";
protected
  Real vx[3,3] "Skew [vb]x";
  Real ax[3,3] "Skew [ab]x";
  Real wx[3,3] "Skew [omega]x";
algorithm
  vx := LieGroups.SO3.Quat.wedge(xi[1:3]);
  ax := LieGroups.SO3.Quat.wedge(xi[4:6]);
  wx := LieGroups.SO3.Quat.wedge(xi[7:9]);

  // ad = {{[omega]x, 0, [vb]x}, {0, [omega]x, [ab]x}, {0, 0, [omega]x}}
  ad := zeros(9, 9);
  ad[1:3, 1:3] := wx;
  ad[1:3, 7:9] := vx;
  ad[4:6, 4:6] := wx;
  ad[4:6, 7:9] := ax;
  ad[7:9, 7:9] := wx;
end small_adjoint;
