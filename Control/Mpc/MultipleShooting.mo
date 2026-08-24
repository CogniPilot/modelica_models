within Control.Mpc;

// SPDX-License-Identifier: Apache-2.0

block MultipleShooting
  "Condensed multiple-shooting transcription, one warm-started step per tick"
  extends PartialTranscription;
protected
  discrete Real nodeStates[ocp.horizonLength, ocp.stateCount](each start=0.0)
    "Shooting-node states x_1..x_N of the current horizon";
  discrete Real controls[ocp.horizonLength, ocp.inputCount](each start=0.0)
    "Interval controls u_0..u_(N-1) of the current horizon";

  function solveTick
    "Warm-shift the horizon, then condense and take one Gauss-Newton step"
    input OcpSpec ocp;
    input Real currentState[:];
    input Real reference[size(currentState, 1)];
    input Real stateWeight[size(currentState, 1)];
    input Real terminalWeight[size(currentState, 1)];
    input Real inputWeight[:];
    input Real controlsPrev[:, size(inputWeight, 1)];
    input Real nodesPrev[size(controlsPrev, 1), size(currentState, 1)];
    input Real stateJacobians[
      size(controlsPrev, 1) * size(currentState, 1), size(currentState, 1)];
    input Real inputJacobians[
      size(controlsPrev, 1) * size(currentState, 1), size(inputWeight, 1)];
    input Real nominalNodes[size(controlsPrev, 1), size(currentState, 1)];
    output Real controlsNext[size(controlsPrev, 1), size(controlsPrev, 2)];
    output Real nodesNext[size(controlsPrev, 1), size(currentState, 1)];
    output Real commandOut[size(controlsPrev, 2)];
    output Boolean solveAccepted;
    output Real cost;
  protected
    Real jacobianMatrix[
      size(controlsPrev, 1) * (size(currentState, 1) + size(controlsPrev, 2)),
      size(controlsPrev, 1) * size(controlsPrev, 2)];
    Real residualVector[
      size(controlsPrev, 1) * (size(currentState, 1) + size(controlsPrev, 2))];
    Real sensitivity[size(currentState, 1),
      size(controlsPrev, 1) * size(controlsPrev, 2)]
      "Running condensed sensitivity dx_k/du of the whole control stack";
    Real sensitivityNew[size(currentState, 1),
      size(controlsPrev, 1) * size(controlsPrev, 2)];
    Real sensitivityAll[size(controlsPrev, 1) * size(currentState, 1),
      size(controlsPrev, 1) * size(controlsPrev, 2)];
    Real carry[size(currentState, 1)]
      "Running condensed defect contribution to dx_k at du = 0";
    Real carryNew[size(currentState, 1)];
    Real carryAll[size(controlsPrev, 1) * size(currentState, 1)];
    Real stateJacobianBlock[size(currentState, 1), size(currentState, 1)];
    Real inputJacobianBlock[size(currentState, 1), size(controlsPrev, 2)];
    Real defect[size(currentState, 1)];
    Real weightRow[size(currentState, 1)];
    Real step[size(controlsPrev, 1) * size(controlsPrev, 2)];
    Real weightSqrt;
    Real accumulator;
    Real stepScale;
    Boolean stepAccepted;
    Integer nx;
    Integer nu;
    Integer horizon;
  algorithm
    nx := size(currentState, 1);
    horizon := size(controlsPrev, 1);
    nu := size(controlsPrev, 2);

    // Warm start: shift the previous solution one interval and repeat the
    // final node. This must stay consistent with the linearization query
    // points published in the equation section, because the provided
    // Jacobians and nominal values are evaluated exactly there.
    controlsNext := controlsPrev;
    nodesNext := nodesPrev;
    for k in 1:(horizon - 1) loop
      controlsNext[k, :] := controlsPrev[k + 1, :];
      nodesNext[k, :] := nodesPrev[k + 1, :];
    end for;

    // Define every working variable before the loops; the checked DAE
    // constructor requires an initial value for everything a loop reads.
    weightSqrt := 0.0;
    accumulator := 0.0;
    stepScale := 0.0;
    stepAccepted := true;
    carry := zeros(size(currentState, 1));
    carryNew := zeros(size(currentState, 1));
    defect := zeros(size(currentState, 1));
    weightRow := zeros(size(currentState, 1));
    stateJacobianBlock := zeros(size(currentState, 1), size(currentState, 1));
    inputJacobianBlock := zeros(size(currentState, 1), size(controlsPrev, 2));
    sensitivity := zeros(size(currentState, 1),
      size(controlsPrev, 1) * size(controlsPrev, 2));
    sensitivityNew := zeros(size(currentState, 1),
      size(controlsPrev, 1) * size(controlsPrev, 2));
    sensitivityAll := zeros(size(controlsPrev, 1) * size(currentState, 1),
      size(controlsPrev, 1) * size(controlsPrev, 2));
    carryAll := zeros(size(controlsPrev, 1) * size(currentState, 1));
    step := zeros(size(controlsPrev, 1) * size(controlsPrev, 2));
    residualVector := zeros(
      size(controlsPrev, 1) * (size(currentState, 1) + size(controlsPrev, 2)));
    jacobianMatrix := zeros(
      size(controlsPrev, 1) * (size(currentState, 1) + size(controlsPrev, 2)),
      size(controlsPrev, 1) * size(controlsPrev, 2));

    // Forward pass over the shooting intervals: condensation of the
    // linearized continuity dx_(k+1) = A_k dx_k + B_k du_k + d_k with
    // dx_0 = 0 into sensitivity and carry, and assembly of the weighted
    // tracking residual and its Jacobian. The residual system is built by
    // accumulation into zeroed arrays and whole-array matrix products;
    // scalar-accumulation loops at this nesting depth are miscompiled by
    // the current compiler, and the matrix form is clearer anyway.
    for k in 1:horizon loop
      // Accumulation into freshly zeroed blocks; a plain elementwise copy
      // loop here is miscompiled by the current compiler.
      stateJacobianBlock := zeros(size(currentState, 1), size(currentState, 1));
      inputJacobianBlock := zeros(size(currentState, 1), size(controlsPrev, 2));
      for i in 1:nx loop
        stateJacobianBlock[i, :] := stateJacobianBlock[i, :]
          + stateJacobians[(k - 1) * nx + i, :];
        inputJacobianBlock[i, :] := inputJacobianBlock[i, :]
          + inputJacobians[(k - 1) * nx + i, :];
      end for;
      defect := nominalNodes[k, :] - nodesNext[k, :];

      carryNew := stateJacobianBlock * carry + defect;
      sensitivityNew := stateJacobianBlock * sensitivity;
      carry := carryNew;
      sensitivity := sensitivityNew;
      for i in 1:nx loop
        for j in 1:nu loop
          sensitivity[i, (k - 1) * nu + j] :=
            sensitivity[i, (k - 1) * nu + j] + inputJacobianBlock[i, j];
        end for;
      end for;

      weightRow := if k == horizon then terminalWeight else stateWeight;
      for i in 1:nx loop
        weightSqrt := sqrt(max(weightRow[i], 0.0));
        carryAll[(k - 1) * nx + i] :=
          carryAll[(k - 1) * nx + i] + carry[i];
        residualVector[(k - 1) * nx + i] :=
          residualVector[(k - 1) * nx + i] + weightSqrt
          * (nodesNext[k, i] + carry[i] - reference[i]);
        for col in 1:(horizon * nu) loop
          sensitivityAll[(k - 1) * nx + i, col] :=
            sensitivityAll[(k - 1) * nx + i, col] + sensitivity[i, col];
          jacobianMatrix[(k - 1) * nx + i, col] :=
            jacobianMatrix[(k - 1) * nx + i, col]
            + weightSqrt * sensitivity[i, col];
        end for;
      end for;
      for j in 1:nu loop
        weightSqrt := sqrt(max(inputWeight[j], 0.0));
        jacobianMatrix[horizon * nx + (k - 1) * nu + j, (k - 1) * nu + j] :=
          jacobianMatrix[horizon * nx + (k - 1) * nu + j, (k - 1) * nu + j]
          + weightSqrt;
        residualVector[horizon * nx + (k - 1) * nu + j] :=
          residualVector[horizon * nx + (k - 1) * nu + j]
          + weightSqrt * controlsNext[k, j];
      end for;
    end for;

    cost := 0.0;
    for row in 1:(horizon * (nx + nu)) loop
      cost := cost + 0.5 * residualVector[row] * residualVector[row];
    end for;

    (step, stepAccepted) := NlpSolver(jacobianMatrix, residualVector,
      ocp.regularization);
    // A rejected step is applied with scale zero instead of branching,
    // because the compiler does not yet accept loops inside conditional
    // branches of a function; the iterate is left unchanged either way.
    stepScale := if stepAccepted then 1.0 else 0.0;
    solveAccepted := stepAccepted;
    for k in 1:horizon loop
      for j in 1:nu loop
        controlsNext[k, j] := controlsNext[k, j]
          + stepScale * step[(k - 1) * nu + j];
      end for;
      for i in 1:nx loop
        accumulator := carryAll[(k - 1) * nx + i];
        for col in 1:(horizon * nu) loop
          accumulator := accumulator
            + sensitivityAll[(k - 1) * nx + i, col] * step[col];
        end for;
        nodesNext[k, i] := nodesNext[k, i] + stepScale * accumulator;
      end for;
    end for;

    commandOut := controlsNext[1, :];
  end solveTick;

equation
  // Query points for the consumer-side linearization: the warm-shifted
  // previous solution, anchored at the sampled current state. Functions of
  // previous-tick values only, so the consumer equations, this block's
  // solve, and the plant close without an algebraic event loop.
  for i in 1:ocp.stateCount loop
    linearizationStates[1, i] = currentState[i];
  end for;
  for k in 2:ocp.horizonLength loop
    for i in 1:ocp.stateCount loop
      linearizationStates[k, i] = pre(nodeStates[k, i]);
    end for;
  end for;
  for k in 1:(ocp.horizonLength - 1) loop
    for j in 1:ocp.inputCount loop
      linearizationControls[k, j] = pre(controls[k + 1, j]);
    end for;
  end for;
  for j in 1:ocp.inputCount loop
    linearizationControls[ocp.horizonLength, j] =
      pre(controls[ocp.horizonLength, j]);
  end for;

algorithm
  when sample(0.0, ocp.samplePeriod) then
    (controls, nodeStates, command, accepted, predictedCost) := solveTick(
      ocp,
      currentState,
      reference,
      stateWeight,
      terminalWeight,
      inputWeight,
      pre(controls),
      pre(nodeStates),
      stateJacobians,
      inputJacobians,
      nominalNodes);
  end when;

  annotation(Documentation(info="<html>
    <p>Multiple shooting with dense condensing, run as a real-time
    iteration: each tick the block shifts the previous horizon one interval
    as a warm start, condenses the linearized continuity constraints
    exactly around it (eliminating the node-state increments through their
    defect and sensitivity), and takes one <code>NlpSolver</code> step on
    the stacked controls. Node states are updated from the same condensed
    linearization, so continuity defects contract as in standard multiple
    shooting; for linear plants they vanish within the step and the tick is
    the exact LQ tracking solution. The first optimized control becomes
    <code>command</code>.</p>
    <p>One linearize-and-step iteration per tick is the classical RTI
    scheme. Running further SQP iterations inside a tick would need fresh
    model evaluations at the stepped iterate, which the linearization ports
    cannot provide within one event; in-tick SQP returns when the
    transcription can call a replaceable prediction function directly (see
    the package notes on the redeclare compiler bug).</p>
    <p>The entire tick is one function call because the compiler does not
    yet accept general loops inside <code>when</code> clauses; see the
    package documentation for all M-A compromises.</p>
  </html>"));
end MultipleShooting;
