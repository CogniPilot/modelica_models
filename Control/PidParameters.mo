within Control;

// SPDX-License-Identifier: Apache-2.0

record PidParameters "Tuning for the shared sampled PID controller"
  Real samplePeriod(unit="s") = 0.02;
  Real kp = 0.0;
  Real ki = 0.0;
  Real kd = 0.0;
  Real integralLimit = 1.0;
  Real commandMin = -1.0;
  Real commandMax = 1.0;
  Real trackingTime(unit="s", min=1.0e-9) = 0.1
    "Applied-command anti-windup tracking time constant";
  Real derivativeCutoffHz(unit="Hz") = 0.0
    "Positive cutoff enables first-order filtering; zero uses the raw derivative";
end PidParameters;
