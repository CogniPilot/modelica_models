within LieGroups.SO3.Quat;
function product_right_factor_jacobian
  "Right-trivialized derivative of product with respect to its right factor"
  input Real q[4] "Left quaternion {w,x,y,z}";
  input Real p[4] "Right quaternion {w,x,y,z}";
  output Real J[3, 3] "d/d(dp) of log_map(product(q,p)^-1 * product(q, p*exp_map(dp)))";
algorithm
  J := identity(3);
  annotation(Documentation(info="<html>
    <p>Rule for <code>product</code>, right slot. A right perturbation of the
    right factor is already a right perturbation of the product, so the rule is
    the identity for every q and p:</p>
    <pre>log_map(product(q,p)^-1 * product(q, p*exp_map(dp))) = dp</pre>
    <p>The identity is exact. The function takes both factors so that every rule
    in the library has its primitive's signature.</p>
  </html>"));
end product_right_factor_jacobian;
