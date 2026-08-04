within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block RateControlAllocator
  "Sampled body-rate control and motor allocation task"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.001;
  parameter Real inertia[3] = {
    0.02166666666666667,
    0.02166666666666667,
    0.04000000000000001};
  parameter Real rateGain[3] = {20.0, 20.0, 10.0};
  parameter Real thrustCoefficient = 8.54858e-6
    "Rotor thrust coefficient [N/(rad/s)^2]";
  parameter Real maxMotorSpeed(unit = "rad/s") = 1100.0;
  parameter Real wrenchToRotorThrust[4, 4] = [
    0.25, -1.4142135623730951, -1.4142135623730951, -15.625;
    0.25, -1.4142135623730951,  1.4142135623730951,  15.625;
    0.25,  1.4142135623730951,  1.4142135623730951, -15.625;
    0.25,  1.4142135623730951, -1.4142135623730951,  15.625];

  Interfaces.RateControlInput inputSignal;
  Interfaces.MotorCommands commands;

protected
  discrete Real angularVelocityMeasuredFlu_rad_s[3](each start = 0.0);
  discrete Real angularVelocityCommandFlu_rad_s[3](each start = 0.0);
  discrete Real momentBodyFlu_Nm[3](each start = 0.0);
  Real allocatedMotorCommand[4];

equation
  allocatedMotorCommand = Control.Multirotor.Allocation.rotorCommands(
    4,
    inputSignal.thrust_N,
    momentBodyFlu_Nm,
    wrenchToRotorThrust,
    fill(thrustCoefficient, 4),
    fill(maxMotorSpeed, 4));
  if inputSignal.armed then
    commands.motor = allocatedMotorCommand;
  else
    commands.motor = zeros(4);
  end if;

algorithm
  when sample(0.0, samplePeriod) then
    // Firmware sensors use FRD; the plant and geometric controller use FLU.
    angularVelocityMeasuredFlu_rad_s := {
      inputSignal.angularVelocityMeasuredFrd_rad_s[1],
      -inputSignal.angularVelocityMeasuredFrd_rad_s[2],
      -inputSignal.angularVelocityMeasuredFrd_rad_s[3]};
    angularVelocityCommandFlu_rad_s :=
      inputSignal.angularVelocitySetpointFlu_rad_s
        + inputSignal.angularVelocityCorrectionFlu_rad_s;
    momentBodyFlu_Nm := Control.Multirotor.RateLoop.bodyMoment(
      angularVelocityCommandFlu_rad_s,
      angularVelocityMeasuredFlu_rad_s,
      inertia,
      rateGain);
  end when;

  annotation(Documentation(info = "<html>
    <p>This block owns the fast RTOS task boundary. It samples the outer-loop
    body-rate and thrust command, converts the firmware FRD gyro measurement
    to the controller's FLU convention, computes body moment, and allocates
    the result to the four motors.</p>
  </html>"));
end RateControlAllocator;
