within LieGroups.SE23.Quat;
function product_left_factor_jacobian
  "Right-trivialized derivative of product with respect to its left factor"
  input Real X1[10] "Left element {p1, v1, q1}";
  input Real X2[10] "Right element {p2, v2, q2}";
  output Real J[9, 9]
    "d/d(dX) of log_map(product(X1,X2)^-1 * product(X1*exp_map(dX), X2))";
algorithm
  J := LieGroups.SE23.Quat.adjoint(LieGroups.SE23.Quat.inverse(X2));
  annotation(Documentation(info="<html>
    <p>Rule for <code>product</code>, left slot, in the right trivialization on
    both sides:</p>
    <pre>log_map(product(X1,X2)^-1 * product(X1*exp_map(dX), X2)) = Ad(X2^-1) dX</pre>
    <p>The identity is exact rather than first order: the perturbation is
    conjugated by X2 and nothing else happens to it. This is the rule that gives
    <code>adjoint</code> its operational meaning, and the reason a right-invariant
    error transported across a group product costs one adjoint and no series.</p>
  </html>"));
end product_left_factor_jacobian;
