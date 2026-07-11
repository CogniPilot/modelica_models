within LieGroups.SE3.Quat;
function small_adjoint "Small adjoint ad_xi for se(3), translation-first ordering"
  input Real xi[6] "Lie algebra {vx,vy,vz, omega_x,omega_y,omega_z}";
  output Real ad[6,6] "6x6 small adjoint matrix";
protected
  Real vx[3,3] "Skew [v]x";
  Real wx[3,3] "Skew [omega]x";
algorithm
  vx := LieGroups.SO3.Quat.wedge(xi[1:3]);
  wx := LieGroups.SO3.Quat.wedge(xi[4:6]);

  // ad = {{[omega]x, [v]x}, {0, [omega]x}}
  ad := zeros(6, 6);
  ad[1:3, 1:3] := wx;
  ad[1:3, 4:6] := vx;
  ad[4:6, 4:6] := wx;
end small_adjoint;
