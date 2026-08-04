within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block ProportionalController
  "Sampled proportional controller with symmetric output limiting"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.005;
  parameter Real gain = 1.0;
  parameter Real outputLimit(min = 0.0) = 1.0;

  Interfaces.FeedbackInput inputSignal;
  Interfaces.FeedbackOutput outputSignal;

algorithm
  when sample(0.0, samplePeriod) then
    outputSignal.error := inputSignal.setpoint - inputSignal.measurement;
    outputSignal.command := MathUtilities.clip(
      gain * outputSignal.error,
      -outputLimit,
      outputLimit);
  end when;
end ProportionalController;
