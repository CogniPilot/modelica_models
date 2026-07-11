within LieGroups.SO3.Quat;
function rotate "Rotate a 3-vector by a quaternion: v' = q @ v"
  input Real q[4] "Unit quaternion {w,x,y,z}";
  input Real v[3] "Input vector";
  output Real v_out[3] "Rotated vector";
protected
  Real R[3,3];
algorithm
  R := LieGroups.SO3.Quat.to_DCM(q);
  v_out := R * v;
end rotate;
