within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block Mixer "Desaturating X-quadrotor mixer"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit="s") = 0.001;

  Interfaces.MixerInput inputSignal;
  Interfaces.MotorCommands commands;

protected
  discrete Real rateCommand[3](each start=0.0);
  discrete Real correction[4](each start=0.0);
  discrete Real maximumIncrease(start=0.0);
  discrete Real maximumDecrease(start=0.0);
  discrete Real increaseScale(start=1.0);
  discrete Real correctionScale(start=1.0);

algorithm
  when sample(0.0, samplePeriod) then
    rateCommand := inputSignal.rateCorrection;
    correction := {
      -rateCommand[1] - rateCommand[2] - rateCommand[3],
      -rateCommand[1] + rateCommand[2] + rateCommand[3],
      rateCommand[1] + rateCommand[2] - rateCommand[3],
      rateCommand[1] - rateCommand[2] + rateCommand[3]
    };
    maximumIncrease := max(
      0.0,
      max(correction[1], max(correction[2], max(correction[3], correction[4]))));
    maximumDecrease := max(
      0.0,
      max(-correction[1], max(-correction[2], max(-correction[3], -correction[4]))));
    if maximumIncrease > 1.0 - inputSignal.throttle
        and maximumIncrease > 0.0 then
      increaseScale := (1.0 - inputSignal.throttle) / maximumIncrease;
    else
      increaseScale := 1.0;
    end if;
    if maximumDecrease > inputSignal.throttle
        and maximumDecrease > 0.0 then
      correctionScale := min(
        increaseScale,
        inputSignal.throttle / maximumDecrease);
    else
      correctionScale := increaseScale;
    end if;
    commands.motor := {
      MathUtilities.clip(
        inputSignal.throttle + correction[1] * correctionScale, 0.0, 1.0),
      MathUtilities.clip(
        inputSignal.throttle + correction[2] * correctionScale, 0.0, 1.0),
      MathUtilities.clip(
        inputSignal.throttle + correction[3] * correctionScale, 0.0, 1.0),
      MathUtilities.clip(
        inputSignal.throttle + correction[4] * correctionScale, 0.0, 1.0)
    };
  end when;
end Mixer;
