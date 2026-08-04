within Control;

// SPDX-License-Identifier: Apache-2.0

record PidCoefficients
  "Coefficients elaborated once before the embedded PID transition runs"
  Real derivativeWeight(min=0.0, max=1.0);
  Real trackingCoefficient(min=0.0, max=1.0);
end PidCoefficients;
