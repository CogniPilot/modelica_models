within LieGroups.SE23.Quat;
function mixed_increment_matrix
  "Increment block N of the closed-form mixed exponential"
  input Real a[9] "Algebra element {vb, ab, omega}";
  input Real B[2, 2] "Coupling matrix as this increment sees it";
  output Real N[3, 2] "Increment block; column 1 is the velocity slot, column 2 the position slot";
protected
  Real omega[3];
  Real theta_sq;
  Real C[3];
  Real Om[3, 3];
  Real Om2[3, 3];
  Real A[3, 2];
  Real I2[2, 2];
algorithm
  omega := a[7:9];
  theta_sq := omega[1]^2 + omega[2]^2 + omega[3]^2;
  C := LieGroups.SE23.Quat.mixed_increment_coefficients(theta_sq);
  Om := LieGroups.SO3.Quat.wedge(omega);
  Om2 := Om * Om;
  A := {{a[4], a[1]}, {a[5], a[2]}, {a[6], a[3]}};
  I2 := identity(2);
  N := A + 0.5 * A * B
    + Om * A * (C[1] * I2 + C[2] * B)
    + Om2 * A * (C[2] * I2 + C[3] * B);
  annotation(Documentation(info="<html>
    <p>N(a, B) = A + A B/2 + Om A (C1 I + C2 B) + Om^2 A (C2 I + C3 B), with
    A = [a_b | v_b] the 3-by-2 acceleration/velocity block of the algebra
    element and Om the skew matrix of its rotation part. This is the increment
    block of the closed-form mixed exponential of Lin, Pant, Perseghetti, and
    Goppert, IEEE L-CSS 2025.</p>
    <p><code>exp_mixed</code> forms the left increment as N(l, B) and the right
    increment as N(r, -B): the sign of the coupling is the only difference
    between them. The derivative rules for <code>exp_mixed</code> are written in
    terms of this block and of
    <code>mixed_increment_matrix_jacobian</code>.</p>
    <p>Pinned against the primitive by evaluating
    <code>exp_mixed</code> at the identity element with a zero right increment,
    where the group product reduces to N(l, B) exactly.</p>
  </html>"));
end mixed_increment_matrix;
