within Control;

// SPDX-License-Identifier: Apache-2.0

package Mpc "Online receding-horizon optimal control as a plain Modelica library"
  annotation(Documentation(info="<html>
    <p>Model-predictive control defined without any language extension. The
    optimal control problem is a record (<code>OcpSpec</code>), the
    transcription of that problem into an online nonlinear least-squares
    step is a replaceable block (<code>PartialTranscription</code>,
    implemented first by <code>MultipleShooting</code>), the plant
    prediction model is a function interface
    (<code>PartialDynamics</code>), and the per-iteration step computation
    is a replaceable function (<code>PartialNlpSolver</code>, implemented
    first by <code>gaussNewtonSqpStep</code>, which needs only first-order
    information and a dense Cholesky solve via
    <code>LinearAlgebra.solveSPD</code>). Selecting a transcription is one
    <code>redeclare</code> line in the consuming model, the same pattern as
    <code>Estimation.StrapdownINS.PartialEstimator</code>.</p>
    <p>Execution is online and clocked: the transcription re-solves inside
    a <code>when sample(0.0, ocp.samplePeriod)</code> clause each control
    tick as a real-time iteration, warm-started by shifting the previous
    solution, and applies the first control of the horizon. The whole solve
    is discrete so an MPC controller composes with sampled estimators and
    plants and remains eligible for embedded code generation.</p>
    <p>Scope of this first milestone (M-A) and deliberate compromises,
    each forced by a current compiler limitation unless noted:</p>
    <ul>
    <li>Costs are quadratic tracking costs with diagonal running, input,
      and terminal weights (design choice); the weights are the diagonals
      of Q, R, and the terminal Q and must be nonnegative (negative entries
      are clamped to zero before the square root). The weight parameters
      live on <code>PartialTranscription</code> instead of inside
      <code>OcpSpec</code> because record components cannot yet be sized by
      sibling record members.</li>
    <li>The problem is unconstrained. Input and state constraint handling
      is the M-B milestone (a constrained QP inner solve); it is not
      emulated here by penalties or clipping (design choice).</li>
    <li>The compiler silently ignores <code>redeclare</code> of functions
      and packages: the constraint is checked, then every call resolves to
      the declared default. The prediction model therefore cannot be a
      replaceable function inside the transcription yet; it enters through
      the linearization ports of <code>PartialTranscription</code>, fed by
      one consumer-side equation calling a per-application
      finite-difference linearization function (see
      <code>Test.doubleIntegratorLinearization</code>).
      <code>NlpSolver</code> keeps the intended replaceable-function form;
      the solver that executes under this compiler is always the declared
      default.</li>
    <li>One linearize-condense-step iteration per tick (the classical RTI
      scheme). In-tick multi-iteration SQP needs fresh model evaluations at
      the stepped iterate, which requires the replaceable prediction
      function above.</li>
    <li>The multiple-shooting normal equations are formed by dense
      condensing, so each Gauss-Newton step solves a dense
      <code>N*nu</code> system. Structure-exploiting banded or Riccati
      factorizations are the M-C milestone (design choice).</li>
    <li>All per-tick work happens inside one Modelica function call because
      the compiler rejects general for-loops inside <code>when</code>
      clauses (event tensor loops must cover every target axis exactly
      once).</li>
    <li>Inside functions, nested loops survive only as reductions: pure
      elementwise-copy inner loops are miscompiled (an internal
      compaction error), loops inside conditional branches are rejected,
      and every variable a loop reads needs a definition before the loop.
      The solver therefore uses whole-array matrix products for the
      condensation recurrences, assembles by accumulation into zeroed
      arrays, and applies a rejected step with scale zero instead of
      branching (mathematically identical in all three cases).</li>
    <li>Redeclared functions must nominally extend their partial interface
      (structural signature equality is not accepted), so every
      implementation extends its <code>Partial*</code> function.</li>
    </ul>
  </html>"));
end Mpc;
