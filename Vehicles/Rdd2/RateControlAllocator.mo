within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block RateControlAllocator
  "Sampled body-rate control and motor allocation task"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.000625;
  parameter Real inertia[3] = {
    0.02166666666666667,
    0.02166666666666667,
    0.04000000000000001};
  parameter Real rateGain[3] = {20.0, 20.0, 10.0};
  parameter Real maximumBodyMoment_Nm[3] = {2.6, 2.6, 0.30}
    "Conservative requested-moment bounds before allocation";
  parameter Real thrustCoefficient = 8.54858e-6
    "Rotor thrust coefficient [N/(rad/s)^2]";
  parameter Real maxMotorSpeed(unit = "rad/s") = 1100.0;
  parameter Vehicles.Rdd2.RotorGeometry rotorGeometry =
    Vehicles.Rdd2.RotorGeometry();
  parameter Real wrenchToRotorThrust[4, 4] =
    Control.Multirotor.Allocation.quadrotorWrenchToThrust(
      rotorGeometry.positionBodyFlu_m,
      rotorGeometry.yawMomentPerThrust_m);

  Interfaces.RateControlInput inputSignal;
  Interfaces.MotorCommands commands;

protected
  discrete Real unboundedMomentBodyFlu_Nm[3](each start = 0.0);
  discrete Real momentBodyFlu_Nm[3](each start = 0.0);
  discrete Real allocatedMotorCommand[4](each start = 0.0, each fixed = true);

equation
  commands.motor = if inputSignal.armed then
      allocatedMotorCommand
    else
      zeros(4);

initial algorithm
  assert(inertia[1] > 0.0 and inertia[2] > 0.0 and inertia[3] > 0.0,
    "Every body inertia must be positive");
  assert(rateGain[1] >= 0.0 and rateGain[2] >= 0.0 and rateGain[3] >= 0.0,
    "Every body-rate gain must be nonnegative");
  assert(thrustCoefficient > 0.0,
    "Every rotor thrust coefficient must be positive");
  assert(maxMotorSpeed > 0.0,
    "Every maximum rotor speed must be positive");

algorithm
  when sample(0.0, samplePeriod) then
    unboundedMomentBodyFlu_Nm := Control.Multirotor.RateLoop.bodyMoment(
      inputSignal.angularVelocityCommandFlu_rad_s,
      inputSignal.angularVelocityMeasuredFlu_rad_s,
      inertia,
      rateGain);
    momentBodyFlu_Nm := {
      MathUtilities.clip(unboundedMomentBodyFlu_Nm[1],
        -maximumBodyMoment_Nm[1], maximumBodyMoment_Nm[1]),
      MathUtilities.clip(unboundedMomentBodyFlu_Nm[2],
        -maximumBodyMoment_Nm[2], maximumBodyMoment_Nm[2]),
      MathUtilities.clip(unboundedMomentBodyFlu_Nm[3],
        -maximumBodyMoment_Nm[3], maximumBodyMoment_Nm[3])};
    allocatedMotorCommand := Control.Multirotor.Allocation.rotorCommands(
      4,
      inputSignal.thrust_N,
      momentBodyFlu_Nm,
      wrenchToRotorThrust,
      fill(thrustCoefficient, 4),
      fill(maxMotorSpeed, 4));
  end when;

  annotation(Documentation(info = "<html>
    <p>This block owns the fast RTOS task boundary. It samples the outer-loop
    body-rate and thrust command, computes body moment in the shared FLU
    convention, and allocates the result to the four motors.</p>
    <p>The model intentionally retains vector and matrix equations. Rumoca
    lowers those equations to bounded scalar Production Code; the source model
    remains compact and auditable.</p>
  </html>"));
end RateControlAllocator;
