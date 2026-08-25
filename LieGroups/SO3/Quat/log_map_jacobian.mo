within LieGroups.SO3.Quat;
function log_map_jacobian
  "Right-trivialized derivative of log_map with respect to its quaternion"
  input Real q[4] "Unit quaternion {w,x,y,z}, the argument of log_map";
  output Real J[3, 3] "d/d(dq) of (log_map(q * exp_map(dq)) - log_map(q)) at dq = 0";
algorithm
  J := LieGroups.SO3.Quat.right_jacobian_inv_exact(LieGroups.SO3.Quat.log_map(q));
  annotation(Documentation(info="<html>
    <p>Rule for <code>log_map</code>. The quaternion slot is perturbed on the
    right and the rotation-vector result is read back additively, so this
    returns the matrix J with</p>
    <pre>log_map(q * exp_map(dq)) - log_map(q) = J*dq + O(|dq|^2)</pre>
    <p>which is the inverse SO(3) right Jacobian J_r(v)^-1 evaluated at
    v = log_map(q).</p>
    <p>The coefficient matrix is <code>right_jacobian_inv_exact</code>, the
    member of the rule-local family that inverts
    <code>right_jacobian_exact</code> exactly at every magnitude, rather than
    <code>right_jacobian_inv</code>, whose two-term series below 0.1 rad leaves
    <code>exp_map_jacobian</code> and this rule non-inverse by 1.1e-8 at
    0.0999 rad. <code>log_map</code> itself is closed form everywhere above
    1e-10, so nothing here is the derivative of a series.</p>
  </html>"));
end log_map_jacobian;
