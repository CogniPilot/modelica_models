within LieGroups.SO3.Quat;
function exp_map_jacobian
  "Right-trivialized derivative of exp_map with respect to its rotation vector"
  input Real v[3] "Rotation vector, the argument of exp_map";
  output Real J[3, 3] "d/d(dv) of log_map(exp_map(v)^-1 * exp_map(v + dv)) at dv = 0";
algorithm
  J := LieGroups.SO3.Quat.right_jacobian(v);
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
    <p>The coefficient matrix is <code>right_jacobian</code>, which exists
    separately because SE(3) and SE_2(3) use it as a block of their own
    exponentials rather than as a derivative.</p>
  </html>"));
end exp_map_jacobian;
