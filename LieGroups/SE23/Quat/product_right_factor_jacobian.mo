within LieGroups.SE23.Quat;
function product_right_factor_jacobian
  "Right-trivialized derivative of product with respect to its right factor"
  input Real X1[10] "Left element {p1, v1, q1}";
  input Real X2[10] "Right element {p2, v2, q2}";
  output Real J[9, 9]
    "d/d(dX) of log_map(product(X1,X2)^-1 * product(X1, X2*exp_map(dX)))";
algorithm
  J := identity(9);
  annotation(Documentation(info="<html>
    <p>Rule for <code>product</code>, right slot. A right perturbation of the
    right factor is already a right perturbation of the product, so the rule is
    the identity for every pair of elements and the identity is exact.</p>
    <p>The function takes both factors so that every rule in the library has its
    primitive's signature.</p>
  </html>"));
end product_right_factor_jacobian;
