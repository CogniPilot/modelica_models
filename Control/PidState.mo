within Control;

// SPDX-License-Identifier: Apache-2.0

record PidState "Numeric state of one sampled PID transition"
  Real integralContribution "Integral contribution in command units";
  Real error "Previous sampled tracking error";
  Real derivative "Filtered error derivative";
end PidState;
