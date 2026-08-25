within LieGroups.SO3.Quat;
function exp_map_jacobian
  "Right-trivialized derivative of exp_map with respect to its rotation vector"
  input Real v[3] "Rotation vector, the argument of exp_map";
  output Real J[3, 3] "d/d(dv) of log_map(exp_map(v)^-1 * exp_map(v + dv)) at dv = 0";
algorithm
  J := LieGroups.SO3.Quat.right_jacobian_exact(v);
  annotation(Documentation(info="<html>
    <p>Rule for <code>exp_map</code>. The rotation-vector slot is additive and
    the group-valued result is read back on the right, so this returns the
    matrix J with</p>
    <pre>log_map(exp_map(v)^-1 * exp_map(v + dv)) = J*dv + O(|dv|^2)</pre>
    <p>which is the SO(3) right Jacobian
    J_r(v) = I - ((1-cos t)/t^2)[v]x + ((t-sin t)/t^3)[v]x^2, t = |v|.
    Equivalently exp_map(v + dv) = exp_map(v) * exp_map(J_r(v) dv) to first
    order, which is the identity that makes J_r the right-trivialized
    derivative.</p>
    <p>The coefficient matrix is <code>right_jacobian_exact</code>, not
    <code>right_jacobian</code>. The two are the same matrix; they differ only
    in where they stop using the closed form. <code>right_jacobian</code> is a
    block of the SE(3) and SE_2(3) exponentials and switches to a two-term
    series at 0.1 rad for the sake of single-precision conditioning in those
    exponentials, while <code>exp_map</code> is closed form above 1e-4 rad. A
    rule that borrowed the 0.1 rad radius would be the derivative of neither
    side and would carry its own truncation, up to |v|^5/720, into every chain
    that multiplies it: 1.3e-8 at 0.0999 rad here, and 2.6e-6 once the
    exp_mixed right-increment rule multiplies it by a 200 m lever arm.</p>
  </html>"));
end exp_map_jacobian;
