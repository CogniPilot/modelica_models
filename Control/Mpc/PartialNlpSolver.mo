within Control.Mpc;

// SPDX-License-Identifier: Apache-2.0

partial function PartialNlpSolver
  "Interface for the per-iteration step solver of the transcribed problem"
  input Real jacobian[:, :]
    "Residual Jacobian with respect to the stacked decision step";
  input Real residual[size(jacobian, 1)] "Residual at the current iterate";
  input Real regularization(min=0.0)
    "Diagonal damping the solver may add where applicable";
  output Real step[size(jacobian, 2)] "Decision-variable step";
  output Boolean accepted "False when the solve must be rejected";
end PartialNlpSolver;
