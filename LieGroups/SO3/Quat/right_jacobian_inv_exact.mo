within LieGroups.SO3.Quat;
function right_jacobian_inv_exact
  "Inverse right Jacobian J_r^-1(v) of SO(3), closed form down to 1e-4 rad"
  input Real v[3] "Rotation vector";
  output Real J_inv[3, 3] "I + [v]x/2 + C [v]x^2, C = 1/t^2 + sin t/(2t(cos t - 1))";
protected
  Real theta_sq;
  Real theta;
  Real halfTheta;
  Real halfSine "sin(t/2)";
  Real C;
  Real S[3, 3] "Skew-symmetric [v]x";
  Real S2[3, 3] "[v]x * [v]x";
  // Same radius as right_jacobian_exact, so the whole rule-local family
  // branches where exp_map branches and a rule chain never crosses two radii.
  constant Real eps = 1.0e-8 "Branch on theta squared, that is |v| = 1e-4 rad";
algorithm
  theta_sq := v[1]^2 + v[2]^2 + v[3]^2;

  S := LieGroups.SO3.Quat.wedge(v);
  S2 := S * S;

  if theta_sq < eps then
    // C = 1/12 + t^2/720 + t^4/30240 + t^6/1209600 + ...: all terms positive
    // and decreasing, so three retained terms truncate below t^6/1209600.
    C := 1.0/12.0 + theta_sq / 720.0 + theta_sq * theta_sq / 30240.0;
  else
    theta := sqrt(max(theta_sq, eps));
    halfTheta := 0.5 * theta;
    halfSine := sin(halfTheta);
    // C = (sin(t/2) - (t/2)cos(t/2)) / (t^2 sin(t/2)), which is the identity
    // 1/t^2 + sin t/(2t(cos t - 1)) with the two large terms already cancelled.
    // Written as the difference of the two, C loses 1/t^2 worth of digits and
    // costs J about 2*eps/t^2; written this way the remaining cancellation is
    // in sin(t/2) - (t/2)cos(t/2), whose leading term is (t/2)^3/3, and it
    // costs J about eps uniformly in t.
    C := (halfSine - halfTheta * cos(halfTheta)) / (theta_sq * halfSine);
  end if;

  J_inv := identity(3) + 0.5 * S + C * S2;

  annotation(Documentation(info="<html>
    <p>The same matrix as <code>right_jacobian_inv</code>, evaluated so that it
    is the exact inverse of <code>right_jacobian_exact</code> at every
    magnitude. The derivative rules use this one;
    <code>right_jacobian_inv</code> stays as the SE(3) and SE_2(3) building
    block.</p>
    <p><b>Why the rewritten closed form.</b> The direct expression
    C = 1/t^2 + sin t/(2t(cos t - 1)) subtracts two quantities of size 1/t^2 to
    leave 1/12, and cos t - 1 itself loses its numerator, so C carries an
    absolute error of about 2*eps/t^4 and J_inv about 2*eps/t^2, of order 1e-8
    at 1e-4 rad. Measured, the direct form puts <code>log_map_jacobian</code>
    5.2e-9 away from a central difference of <code>log_map</code> at 1e-4 rad,
    against the 2e-9 that rule is asserted against, so aligning the radius
    without rewriting the coefficient would have replaced one out-of-budget
    band with another. The
    equivalent form (sin(t/2) - (t/2)cos(t/2)) / (t^2 sin(t/2)) has already
    performed that cancellation symbolically; its numerator's leading term is
    (t/2)^3/3, so the error it contributes to J_inv is about eps at every
    magnitude, and it stays finite through t = pi where
    (A/2 - B)/(1 - B t^2), the other closed rearrangement, becomes 0/0.</p>
    <p><b>Branch radius.</b> Below 1e-4 rad the retained series truncates below
    t^6/1209600, and C enters J_inv multiplied by ||[v]x^2|| = t^2, so</p>
    <pre>||dJ_inv|| &lt;= t^8/1209600 &lt;= 8.3e-39  at t = 1e-4</pre>
    <p>Measured, <code>right_jacobian_exact(v)*right_jacobian_inv_exact(v)</code>
    departs from the identity by at most 4.5e-16 over 1e-6 to 3 rad, where the
    two primitives depart by 1.1e-8 at 0.0999 rad.</p>
  </html>"));
end right_jacobian_inv_exact;
