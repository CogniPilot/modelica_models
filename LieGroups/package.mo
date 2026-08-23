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
      primitive agree to the finite-difference floor. The one exception is the
      branch radius itself: <code>exp_mixed</code> and the SO(3) Jacobians switch
      between a retained series and the closed form at
      <code>theta^2 = 1e-2</code>, and the two sides differ there by about
      1.4e-7 in the leading coefficient, so a finite difference whose step
      straddles that radius is not a valid oracle for any rule.</li>
    </ul>
    <p>The <code>exp_mixed</code> rules are the first variation of the
    closed-form mixed exponential of Lin, Pant, Perseghetti, and Goppert,
    &quot;On Closed-Form Preintegration for a Class of Mixed-Invariant Systems in
    SE_n(3)&quot;, IEEE L-CSS, 2025, and are themselves closed form: no series in
    the increment, no matrix exponential, and no quadrature.</p>
  </html>"));
end LieGroups;
