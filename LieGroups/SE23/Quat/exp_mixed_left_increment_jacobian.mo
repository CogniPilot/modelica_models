within LieGroups.SE23.Quat;
function exp_mixed_left_increment_jacobian
  "Right-trivialized derivative of exp_mixed with respect to its left increment"
  input Real X0[10] "Initial state {p0, v0, q0}";
  input Real l[9] "Left (body-frame) algebra element";
  input Real r[9] "Right (world-frame) algebra element";
  input Real B[2, 2] "Coupling matrix";
  output Real J[9, 9]
    "d/d(dl) of log_map(X1^-1 * exp_mixed(X0, l + dl, r, B)), X1 = exp_mixed(X0, l, r, B)";
protected
  Real DN[6, 9];
  Real RlTransposed[3, 3];
algorithm
  DN := LieGroups.SE23.Quat.mixed_increment_matrix_jacobian(l, B);
  RlTransposed := transpose(LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.exp_map(l[7:9])));
  J := zeros(9, 9);
  J[1:3, :] := RlTransposed * DN[4:6, :];
  J[4:6, :] := RlTransposed * DN[1:3, :];
  J[7:9, 7:9] := LieGroups.SO3.Quat.right_jacobian_exact(l[7:9]);
  annotation(Documentation(info="<html>
    <p>Rule for <code>exp_mixed</code>, left-increment slot. The increment is a
    plain algebra vector and is perturbed additively; the result is read back on
    the right at X1 = exp_mixed(X0, l, r, B), so this returns J with</p>
    <pre>log_map(X1^-1 * exp_mixed(X0, l + dl, r, B)) = J*dl + O(|dl|^2)</pre>
    <p>Closed form. The left increment enters the result only through its own
    rotation R_l and increment block N_l, and both the initial state and the
    right increment cancel out of a right-trivialized error, leaving</p>
    <pre>[e_v | e_p] = R_l^T dN_l
e_omega     = J_r(omega_l) d(omega_l)</pre>
    <p>with dN_l supplied by
    <code>mixed_increment_matrix_jacobian</code> and J_r the SO(3) right
    Jacobian. This is the rule that carries inertial-bias sensitivity into a
    retrodicted or preintegrated pose, because the bias enters the propagation
    exactly by shifting this increment.</p>
    <p>J_r here is <code>right_jacobian_exact</code>. The increment reaches the
    result through <code>exp_map</code>, which is closed form above 1e-4 rad, so
    the rotation block of this rule is closed form there too;
    <code>right_jacobian</code>, whose series radius is 0.1 rad, would put its
    own truncation of order |omega|^5/720 into this block, 1.3e-8 at 0.0999 rad.
    The increment block N_l keeps <code>mixed_increment_coefficients</code>,
    whose 0.1 rad radius is <code>exp_mixed</code>'s own: there the rule and the
    primitive branch together, and their series cancel exactly.</p>
  </html>"));
end exp_mixed_left_increment_jacobian;
