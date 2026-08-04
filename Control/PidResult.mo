within Control;

// SPDX-License-Identifier: Apache-2.0

record PidResult "Algebraic outputs of one sampled PID transition"
  Real unconstrainedCommand;
  Real command;
  Real preview "Integral-free command";
end PidResult;
