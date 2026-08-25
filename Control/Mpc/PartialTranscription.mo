within Control.Mpc;

// SPDX-License-Identifier: Apache-2.0

partial block PartialTranscription
  "Common replaceable interface for online receding-horizon transcriptions"
  parameter OcpSpec ocp = OcpSpec();
  parameter Real stateWeight[ocp.stateCount] = fill(1.0, ocp.stateCount)
    "Diagonal of the running state tracking weight Q";
  parameter Real inputWeight[ocp.inputCount] = fill(1.0, ocp.inputCount)
    "Diagonal of the running control effort weight R";
  parameter Real terminalWeight[ocp.stateCount] = fill(1.0, ocp.stateCount)
    "Diagonal of the terminal state tracking weight";
  replaceable function NlpSolver = Control.Mpc.gaussNewtonSqpStep
    constrainedby Control.Mpc.PartialNlpSolver
    "Per-iteration step solver for the transcribed problem";

  input Real currentState[ocp.stateCount] "Measured or estimated plant state";
  input Real reference[ocp.stateCount] "Tracking setpoint for the horizon";
  input Real stateJacobians[ocp.horizonLength * ocp.stateCount, ocp.stateCount]
    "Stacked A_k = df/dx of the prediction model at the query points";
  input Real inputJacobians[ocp.horizonLength * ocp.stateCount, ocp.inputCount]
    "Stacked B_k = df/du of the prediction model at the query points";
  input Real nominalNodes[ocp.horizonLength, ocp.stateCount]
    "f(x, u) of the prediction model at the query points";

  output Real linearizationStates[ocp.horizonLength, ocp.stateCount]
    "States at which the consumer must evaluate and linearize its model";
  output Real linearizationControls[ocp.horizonLength, ocp.inputCount]
    "Controls at which the consumer must evaluate and linearize its model";
  discrete output Real command[ocp.inputCount](each start=0.0)
    "First control of the optimized horizon, held between ticks";
  discrete output Boolean accepted(start=true)
    "The step solve of the last tick was accepted";
  discrete output Real predictedCost(start=0.0)
    "Half squared residual norm at the last linearization point";

  annotation(Documentation(info="<html>
    <p>Every transcription re-solves its optimal control problem once per
    <code>ocp.samplePeriod</code> tick from the sampled
    <code>currentState</code>, warm-started from its previous solution, and
    exposes the first optimized control as <code>command</code>.</p>
    <p>The prediction model enters through the linearization ports: the
    transcription publishes the horizon points it will linearize around
    (<code>linearizationStates</code>, <code>linearizationControls</code>,
    both functions of previous-tick values only), and the consumer feeds
    back the model evaluations and Jacobians there, normally with one
    equation calling a finite-difference linearization function built on
    its <code>PartialDynamics</code> implementation (see
    <code>Control.Mpc.Test.doubleIntegratorLinearization</code> for the
    reference pattern). This port indirection
    exists because the compiler currently ignores <code>redeclare</code> of
    functions and packages (it validates the constraint, then silently
    resolves calls to the declared default), so a replaceable prediction
    function inside the transcription cannot be selected yet. When function
    redeclaration lands, a <code>replaceable PartialDynamics</code> slot
    should replace these ports and the per-application linearization
    functions.</p>
    <p><code>NlpSolver</code> is that intended pattern, declared today: the
    executed solver under the current compiler is always the declared
    default (<code>gaussNewtonSqpStep</code>); the redeclare line is honored
    by OpenModelica and becomes selectable here once the compiler bug is
    fixed.</p>
  </html>"));
end PartialTranscription;
