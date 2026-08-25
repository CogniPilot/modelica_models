within Control.Mpc;

// SPDX-License-Identifier: Apache-2.0

partial function PartialDynamics
  "Interface for the discrete-time prediction model x+ = f(x, u)"
  input Real state[:] "State at the start of the interval";
  input Real controlInput[:] "Control held over the interval";
  input Real samplePeriod(unit="s") "Interval length";
  output Real nextState[size(state, 1)] "State at the end of the interval";
end PartialDynamics;
