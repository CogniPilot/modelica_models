within LieGroups.SE23.Quat;
function adjoint "Adjoint Ad_X for SE_2(3)"
  input Real X[10] "{p, v, q}";
  output Real Ad[9,9] "9x9 adjoint matrix";
protected
  Real R[3,3];
  Real px[3,3] "Skew [p]x";
  Real vx[3,3] "Skew [v]x";
  Real pR[3,3] "[p]x * R";
  Real vR[3,3] "[v]x * R";
algorithm
  R := LieGroups.SO3.Quat.to_DCM(X[7:10]);
  px := LieGroups.SO3.Quat.wedge(X[1:3]);
  vx := LieGroups.SO3.Quat.wedge(X[4:6]);

  pR := px * R;
  vR := vx * R;

  // Ad = {{R, 0, [p]x*R}, {0, R, [v]x*R}, {0, 0, R}}
  Ad := zeros(9, 9);
  Ad[1:3, 1:3] := R;
  Ad[1:3, 7:9] := pR;
  Ad[4:6, 4:6] := R;
  Ad[4:6, 7:9] := vR;
  Ad[7:9, 7:9] := R;
end adjoint;
