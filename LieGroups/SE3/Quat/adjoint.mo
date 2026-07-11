within LieGroups.SE3.Quat;
function adjoint "Adjoint Ad_X for SE(3), translation-first ordering"
  input Real X[7] "{px,py,pz, qw,qx,qy,qz}";
  output Real Ad[6,6] "6x6 adjoint matrix";
protected
  Real R[3,3] "Rotation matrix";
  Real px[3,3] "Skew [p]x";
  Real pR[3,3] "[p]x * R";
algorithm
  R := LieGroups.SO3.Quat.to_DCM(X[4:7]);

  // Skew of position
  px := LieGroups.SO3.Quat.wedge(X[1:3]);

  pR := px * R;

  // Ad = {{R, [p]x*R}, {0, R}}
  Ad := zeros(6, 6);
  Ad[1:3, 1:3] := R;
  Ad[1:3, 4:6] := pR;
  Ad[4:6, 4:6] := R;
end adjoint;
