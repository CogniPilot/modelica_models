within;
package LieGroups "Lie group library for rigid body mechanics"
  annotation(
    uses(LinearAlgebra),
    Documentation(info="<html>
    <p>Modelica implementation of Lie group operations for SO(3) and related groups.</p>
    <p>Quaternion convention: q = {w, x, y, z} (scalar first, Hamilton).</p>

    <h4>Derivative rules</h4>
    <p>Every primitive in this library that is differentiated by downstream
    estimators carries a companion <i>rule function</i> giving its derivative in
    closed form. The rules obey one convention throughout.</p>
    <ul>
      <li><b>Naming.</b> A rule is named
      <code>&lt;primitive&gt;_&lt;slot&gt;_jacobian</code> and lives in the same
      package as the primitive it differentiates. <code>&lt;primitive&gt;</code>
      is the exact name of the differentiated function and <code>&lt;slot&gt;</code>
      names the input being varied; the slot is omitted when the primitive has a
      single input. A rule takes exactly the primitive's own inputs and returns
      the derivative evaluated at that point.</li>

      <li><b>Trivialization: right, everywhere.</b> A group-valued slot is
      perturbed on the right, <code>X -&gt; X * exp_map(d)</code>, and a
      group-valued result is read back on the right,
      <code>d_out = log_map(Y^(-1) * Y_perturbed)</code>. So for
      <code>Y = f(X)</code> the rule returns the matrix <code>J</code> with
      <code>log_map(f(X)^(-1) * f(X * exp_map(d))) = J*d + O(|d|^2)</code>.
      This matches the local, body-frame error coordinate the strapdown
      estimators carry and the trivialization <code>exp_mixed</code> is written
      in. Left-trivialized quantities are never returned by a rule; where a left
      Jacobian is needed as a building block it is spelled
      <code>left_jacobian</code> and is not a rule.</li>

      <li><b>Vector slots are additive.</b> A slot that is a plain vector or
      matrix, such as the rotation vector of <code>exp_map</code> or the algebra
      increments of <code>exp_mixed</code>, is perturbed additively,
      <code>a -&gt; a + d</code>. A plain-vector <i>result</i> is likewise read
      back additively.</li>

      <li><b>Rules differentiate the implementation.</b> A rule is the derivative
      of the function as this library computes it, including its retained
      small-angle series, so a rule and a central finite difference of its
      primitive agree to the finite-difference floor. A rule therefore has to
      branch where its primitive branches, not where some other function does.
      <code>exp_map</code> is closed form above <code>theta = 1e-4</code>, while
      <code>right_jacobian</code> and its relatives switch to a two-term series
      at <code>theta = 0.1</code> because they are blocks of the SE(3) and
      SE_2(3) exponentials that flight code evaluates in single precision. The
      rules therefore use a separate rule-local family,
      <code>right_jacobian_exact</code> and its three siblings, identical in
      value and branching at <code>theta = 1e-4</code> like
      <code>exp_map</code>. A rule built on the 0.1 rad radius instead carries
      its own truncation, of order <code>|v|^5/720</code>, into everything that
      multiplies it: 1.3e-8 at 0.0999 rad for
      <code>exp_map_jacobian</code>, and 2.6e-6 once
      <code>exp_mixed_right_increment_jacobian</code> multiplies it by a 200 m
      position lever arm.</li>

      <li><b>Where a finite difference is not an oracle.</b> A difference is
      invalid only where its step straddles a branch <i>inside the primitive it
      differences</i>. That happens for <code>exp_mixed</code>, whose own
      coefficients switch at <code>theta = 0.1</code> with the two sides
      differing by about 1.4e-7 in the leading coefficient: a 1e-6 step
      straddling that radius divides the jump by 2e-6 and reports 7e-2 no matter
      how exact the rule is, so the exp_mixed sweeps skip |omega| = 0.1 and only
      that magnitude. It does not happen for the SO(3) rules: their differences
      call <code>exp_map</code>, <code>log_map</code>, <code>product</code>,
      <code>inverse</code> and <code>rotate</code>, none of which branches at
      0.1 rad, and the 1e-4 rad branch they do cross has two sides differing by
      about 1e-33.</li>
    </ul>
    <p>The <code>exp_mixed</code> rules are the first variation of the
    closed-form mixed exponential of Lin, Pant, Perseghetti, and Goppert,
    &quot;On Closed-Form Preintegration for a Class of Mixed-Invariant Systems in
    SE_n(3)&quot;, IEEE L-CSS, 2025, and are themselves closed form: no series in
    the increment, no matrix exponential, and no quadrature.</p>
  </html>"));
end LieGroups;
