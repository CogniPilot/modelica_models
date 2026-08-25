within LieGroups.SO3.Quat;
function left_jacobian_exact
  "Left Jacobian J_l(v) of SO(3), closed form down to the exp_map branch radius"
  input Real v[3] "Rotation vector";
  output Real J[3, 3] "I + ((1-cos t)/t^2)[v]x + ((t-sin t)/t^3)[v]x^2, t = |v|";
algorithm
  J := LieGroups.SO3.Quat.right_jacobian_exact(-v);
  annotation(Documentation(info="<html>
    <p>The same matrix as <code>left_jacobian</code>, evaluated with the branch
    radius and the coefficient forms of
    <code>right_jacobian_exact</code>. Both coefficients depend on the rotation
    vector only through t^2, so J_l(v) = J_r(-v) is exact rather than an
    approximation, and reflecting is the whole implementation.</p>
    <p>No rule returns a left-trivialized derivative, so no rule calls this
    today; <code>Tests.RuleChecks.so3JacobianFamilyResiduals</code> does, against
    the transpose of <code>right_jacobian_exact</code>. It exists because the
    reflection is the definition of the left Jacobian and a left-trivialized
    rule added later must not reach for <code>left_jacobian</code>, whose
    0.1 rad radius would put the same |v|^5/720 series truncation back into a
    derivative.</p>
  </html>"));
end left_jacobian_exact;
