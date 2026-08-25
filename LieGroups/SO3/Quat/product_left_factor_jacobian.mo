within LieGroups.SO3.Quat;
function product_left_factor_jacobian
  "Right-trivialized derivative of product with respect to its left factor"
  input Real q[4] "Left quaternion {w,x,y,z}";
  input Real p[4] "Right quaternion {w,x,y,z}";
  output Real J[3, 3] "d/d(dq) of log_map(product(q,p)^-1 * product(q*exp_map(dq), p))";
algorithm
  J := transpose(LieGroups.SO3.Quat.to_DCM(p));
  annotation(Documentation(info="<html>
    <p>Rule for <code>product</code>, left slot. Both the perturbed slot and the
    group-valued result use the right trivialization, so</p>
    <pre>log_map(product(q,p)^-1 * product(q*exp_map(dq), p)) = J*dq</pre>
    <p>with J = Ad(p^-1) = R(p)^T. The identity is exact, not first order:
    conjugation by p carries the perturbation unchanged apart from the frame
    change, so no series and no evaluation of q enter the result.</p>
  </html>"));
end product_left_factor_jacobian;
