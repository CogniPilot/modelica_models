within Estimation.StrapdownINS.ESKF;

function considerGainProjector
  "Consider-projector for the rotation rows of a Kalman gain"
  input Real axis[3]
    "Zero leaves the gain alone; a UNIT body axis projects the rotation rows
     onto that axis";
  output Real projector[3, 3]
    "n*n' for a unit axis, the identity for the zero default";
algorithm
  projector := if axis * axis > 0.5
    then {{axis[1] * axis[1], axis[1] * axis[2], axis[1] * axis[3]},
          {axis[2] * axis[1], axis[2] * axis[2], axis[2] * axis[3]},
          {axis[3] * axis[1], axis[3] * axis[2], axis[3] * axis[3]}}
    else identity(3);
  annotation(
    Inline = false,
    Documentation(info = "<html>
    <p>The projector is one total definition over the WHOLE matrix, so the
    checked DAE lowering sees a single conditional rather than nine
    element-wise ones.</p>
    <p>IT LIVES IN ITS OWN FUNCTION FOR AN EMISSION REASON, not a modelling
    one. Written inline in <code>correctLinear</code>, this conditional is a
    matrix-valued expression that feeds the gain contraction, and the compiler
    fuses it into that contraction's loop nest: every element instantiates the
    whole guard plus a ternary chain selecting which product the computed index
    names. Measured on
    <code>Vehicles.Rdd2.NavigationEstimator --target galec-production</code>,
    that fusion took each <code>correctLinear</code> specialization from 669 to
    about 22,200 lines of emitted C, and there are four specializations. A
    function output is materialized instead of fused, so the nine elements are
    computed once here and the callers contract against a plain 3x3.</p>
    <p>The arithmetic is byte-for-byte the expression it replaces: same
    products, same operand order, same <code>identity(3)</code> default. This
    is the emission shape only; the Schmidt/consider treatment it serves is
    unchanged and documented at its use site.</p>
    <p><code>Inline = false</code> is the barrier, not the function boundary
    alone. The eFMI production path already refuses to substitute this call
    twice over: a packaged container admits only a no-inline emission policy,
    and a certifiable projection refuses to substitute an aggregate result at
    all. Neither of those is a promise to the model. A container-free target
    that permits scalarization could legally substitute the call and replicate
    the conditional per element again. The annotation is checked ahead of every
    policy, so the requirement is stated in the model rather than left to the
    target's configuration. It binds this compiler only; no Modelica helper is
    an optimization barrier across tools.</p>
  </html>"));
end considerGainProjector;
