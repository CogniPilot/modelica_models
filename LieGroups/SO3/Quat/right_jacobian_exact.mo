within LieGroups.SO3.Quat;
function right_jacobian_exact
  "Right Jacobian J_r(v) of SO(3), closed form down to the exp_map branch radius"
  input Real v[3] "Rotation vector";
  output Real J[3, 3] "I - ((1-cos t)/t^2)[v]x + ((t-sin t)/t^3)[v]x^2, t = |v|";
protected
  Real theta_sq;
  Real theta;
  Real halfTheta;
  Real halfSinc "sin(t/2)/(t/2)";
  Real A "(1 - cos t) / t^2";
  Real B "(t - sin t) / t^3";
  Real S[3, 3] "Skew-symmetric [v]x";
  Real S2[3, 3] "[v]x * [v]x";
  // exp_map is closed form above |v| = 1e-4 rad, so its derivative is too. The
  // primitive right_jacobian stops at 0.1 rad because it is a block of the SE(3)
  // and SE_2(3) exponentials that flight code evaluates in single precision;
  // that radius is a conditioning choice about those exponentials, not about
  // any derivative, and a rule that inherited it would report its own series
  // truncation, up to |v|^5/720, as a disagreement with the primitive.
  constant Real eps = 1.0e-8 "Branch on theta squared, that is |v| = 1e-4 rad";
algorithm
  theta_sq := v[1]^2 + v[2]^2 + v[3]^2;

  S := LieGroups.SO3.Quat.wedge(v);
  S2 := S * S;

  if theta_sq < eps then
    // Three retained terms of A = sum (-1)^k t^(2k)/(2k+2)! and
    // B = sum (-1)^k t^(2k)/(2k+3)!. Both alternate with decreasing terms, so
    // the truncation is bounded by the first dropped term: |dA| <= t^6/40320
    // and |dB| <= t^6/362880.
    A := 0.5 - theta_sq / 24.0 + theta_sq * theta_sq / 720.0;
    B := 1.0/6.0 - theta_sq / 120.0 + theta_sq * theta_sq / 5040.0;
  else
    // NaN-safe denominator: max(theta_sq, eps) is exact when this branch is
    // taken and keeps the closed form finite under both-branch evaluation.
    theta := sqrt(max(theta_sq, eps));
    halfTheta := 0.5 * theta;
    // (1 - cos t)/t^2 = (1/2)(sin(t/2)/(t/2))^2. The half-angle form subtracts
    // nothing, so A keeps full relative precision at every magnitude, while
    // 1 - cos t written directly loses its numerator as cos t approaches one
    // and costs the result about eps/t: at the branch radius the direct form
    // contributes 2.6e-13 to J where this one contributes 5.6e-21.
    halfSinc := sin(halfTheta) / halfTheta;
    A := 0.5 * halfSinc * halfSinc;
    // t - sin t does cancel, but B multiplies [v]x^2, whose norm is t^2, so the
    // cancellation contributes about eps to J at every magnitude rather than
    // growing as t falls.
    B := (theta - sin(theta)) / (theta * theta * theta);
  end if;

  J := identity(3) - A * S + B * S2;

  annotation(Documentation(info="<html>
    <p>The same matrix as <code>right_jacobian</code>,
    J_r(v) = I - ((1-cos t)/t^2)[v]x + ((t-sin t)/t^3)[v]x^2 with t = |v|,
    evaluated in closed form wherever <code>exp_map</code> is closed form. The
    derivative rules use this one; the SE(3) and SE_2(3) exponentials keep
    <code>right_jacobian</code>.</p>
    <p><b>Branch radius.</b> <code>exp_map</code> switches from its retained
    series to the closed form at |v| = 1e-4 rad, so above that radius the group
    element it returns is the true exponential and the true right-trivialized
    derivative of that element is the closed-form J_r. Below the radius this
    function retains three series terms, and their truncation bounds
    |dA| &lt;= t^6/40320 and |dB| &lt;= t^6/362880 give</p>
    <pre>||dJ|| &lt;= |dA| ||[v]x|| + |dB| ||[v]x^2||
       &lt;= t^7/40320 + t^8/362880
       &lt;= 2.5e-33  at the branch radius t = 1e-4</pre>
    <p>which is seventeen orders below binary64 rounding, so the branch is
    invisible: a central difference whose step straddles t = 1e-4 sees a jump of
    that size rather than the 1.4e-7 jump the 0.1 rad radius produces.</p>
    <p>Rounding is bounded the same way. The half-angle form of A carries full
    relative precision, and B's cancellation enters J multiplied by
    ||[v]x^2|| = t^2, so both contribute about eps to J uniformly in t. In
    binary64 the direct form of A would also have stayed inside every budget
    this library asserts, at 2.6e-13 at the branch radius against 5.6e-21 here;
    the half-angle form costs nothing and is what keeps the same statement true
    when this is lowered to single precision, where 1 - cos t is exactly zero
    below about 3e-4 rad.
    Measured against a central difference of <code>exp_map</code>, this function
    sits at the difference floor from 1e-6 to 3 rad, including the 0.09 to 0.11
    band where <code>right_jacobian</code>'s two-term series reaches 1.3e-8.</p>
  </html>"));
end right_jacobian_exact;
