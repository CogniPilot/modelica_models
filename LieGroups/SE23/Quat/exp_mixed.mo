within LieGroups.SE23.Quat;
function exp_mixed "Mixed exponential for SE_2(3) INS propagation"
  input Real X0[10] "Initial state {p0, v0, q0}";
  input Real l[9] "Body-frame algebra element {vb, ab, omega_l}, composed on the right";
  input Real r[9] "World-frame algebra element {vb_r, ab_r, omega_r}, composed on the left";
  input Real B[2,2] "Square-zero coupling matrix (typically {{0,dt},{0,0}})";
  output Real X1[10] "Updated state {p1, v1, q1}";
protected
  Real Nl[3,2];
  Real Nr[3,2];
  Real q_r[4];
  Real q_r0[4];
  Real q1[4];
  Real P0[3,2];
  Real P1[3,2];
algorithm
  // One coefficient implementation serves both increments and their analytic
  // derivatives. The opposite time blocks keep the state in SE_2(3).
  Nl := LieGroups.SE23.Quat.mixed_increment_matrix(l, B);
  Nr := LieGroups.SE23.Quat.mixed_increment_matrix(r, -B);
  q_r := LieGroups.SO3.Quat.exp_map(r[7:9]);
  q_r0 := LieGroups.SO3.Quat.product(q_r, X0[7:10]);
  q1 := LieGroups.SO3.Quat.product(q_r0,
    LieGroups.SO3.Quat.exp_map(l[7:9]));
  P0 := transpose({X0[4:6], X0[1:3]});
  P1 := LieGroups.SO3.Quat.to_DCM(q_r0) * Nl
    + (LieGroups.SO3.Quat.to_DCM(q_r) * P0 + Nr) * (identity(2) + B);
  X1 := cat(1, P1[:,2], P1[:,1], q1);
end exp_mixed;
