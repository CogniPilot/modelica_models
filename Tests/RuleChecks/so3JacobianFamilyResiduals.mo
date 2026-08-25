within Tests.RuleChecks;
function so3JacobianFamilyResiduals
  "How far the rule-local SO(3) Jacobian family is from its own identities"
  input Real angleScales[:] "Rotation magnitudes to sweep";
  input Integer trials "Randomized directions per magnitude";
  output Real worst[3]
    "1: |J_r J_r^-1 - I|, 2: |J_l - transpose(J_r)|, 3: |J_l^-1 - transpose(J_r^-1)|";
protected
  Real draw[3];
  Real axis[3];
  Real axisNorm;
  Real v[3];
  Real forward[3, 3];
  Real inverted[3, 3];
  // Scalar accumulators: see the note in so3RuleResiduals.
  Real worstInverse;
  Real worstLeft;
  Real worstLeftInverse;
algorithm
  worstInverse := 0.0;
  worstLeft := 0.0;
  worstLeftInverse := 0.0;
  for i in 1:size(angleScales, 1) loop
    for trial in 1:trials loop
      // Same seeds as so3RuleResiduals, so the two checks look at the same
      // rotation vectors and a failure of one can be read against the other.
      draw := Tests.RuleChecks.pseudoRandom(7919 * trial + 13, 3);
      axis := draw;
      axisNorm := sqrt(axis[1]^2 + axis[2]^2 + axis[3]^2);
      v := (angleScales[i] / max(axisNorm, 1.0e-12)) * axis;

      forward := LieGroups.SO3.Quat.right_jacobian_exact(v);
      inverted := LieGroups.SO3.Quat.right_jacobian_inv_exact(v);
      worstInverse := max(worstInverse, Tests.Assertions.maxAbsMatrix(
        forward * inverted - identity(3)));
      worstLeft := max(worstLeft, Tests.Assertions.maxAbsMatrix(
        LieGroups.SO3.Quat.left_jacobian_exact(v) - transpose(forward)));
      worstLeftInverse := max(worstLeftInverse, Tests.Assertions.maxAbsMatrix(
        LieGroups.SO3.Quat.left_jacobian_inv_exact(v) - transpose(inverted)));
    end for;
  end for;
  worst := {worstInverse, worstLeft, worstLeftInverse};
  annotation(Documentation(info="<html>
    <p>Three identities the rule-local SO(3) Jacobian family satisfies exactly,
    checked without a finite difference so that the measurement is limited by
    the functions rather than by an oracle.</p>
    <ul>
      <li>J_r(v) J_r^-1(v) = I. This is the identity
      <code>exp_map_jacobian</code> and <code>log_map_jacobian</code> rely on
      when a chain composes one against the other, and it is the sharpest
      statement about the coefficients: it holds to 4.5e-16 across 1e-6 to
      3 rad, where the two primitives it replaces depart by 1.1e-8 at
      0.0999 rad. Both of those keep two series terms, but the residual is not
      symmetric in them: J_r's dropped term sits on the coefficient of [v]x and
      reaches the product multiplied by |v|, giving |v|^5/720, while J_r^-1's
      sits on the coefficient of [v]x^2 and arrives multiplied by |v|^2, giving
      |v|^6/30240, some four hundred times smaller at that magnitude.</li>
      <li>J_l(v) = J_r(v)^T. Transposition flips the sign of the [v]x term and
      leaves the [v]x^2 term, which is exactly the difference between the two
      Jacobians, while the implementation reaches the same matrix by evaluating
      J_r at -v. Both routes negate one skew matrix and leave its square, so the
      identity holds bitwise and measures zero; the entry is a guard against a
      sign or a slot going wrong, not a measurement of accuracy.</li>
      <li>J_l^-1(v) = J_r^-1(v)^T, for the same reason and with the same
      zero.</li>
    </ul>
  </html>"));
end so3JacobianFamilyResiduals;
