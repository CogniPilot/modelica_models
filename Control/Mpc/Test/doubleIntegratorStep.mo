within Control.Mpc.Test;

// SPDX-License-Identifier: Apache-2.0

function doubleIntegratorStep
  "Exact zero-order-hold discretization of a double integrator"
  extends Control.Mpc.PartialDynamics;
algorithm
  nextState := {
    state[1] + samplePeriod * state[2]
      + 0.5 * samplePeriod * samplePeriod * controlInput[1],
    state[2] + samplePeriod * controlInput[1]};
end doubleIntegratorStep;
