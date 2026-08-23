within LieGroups.SO3.Quat;
function left_jacobian_inv_exact
  "Inverse left Jacobian J_l^-1(v) of SO(3), closed form down to 1e-4 rad"
  input Real v[3] "Rotation vector";
  output Real J_inv[3, 3] "I - [v]x/2 + C [v]x^2";
algorithm
  J_inv := LieGroups.SO3.Quat.right_jacobian_inv_exact(-v);
  annotation(Documentation(info="<html>
    <p>The same matrix as <code>left_jacobian_inv</code>, evaluated with the
    branch radius and the coefficient form of
    <code>right_jacobian_inv_exact</code>. C depends on the rotation vector only
    through t^2, so J_l^-1(v) = J_r^-1(-v) is exact.</p>
  </html>"));
end left_jacobian_inv_exact;
