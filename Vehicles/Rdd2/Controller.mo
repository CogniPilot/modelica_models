within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block Controller
  "Composition of the RDD2 pilot mapping, diagnostic PID, and motor mixer"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.001;

  parameter Real kp = 0.12 "Diagnostic rate-axis proportional gain";
  parameter Real ki = 0.35 "Diagnostic rate-axis integral gain";
  parameter Real kd = 0.0015 "Diagnostic rate-axis derivative gain";
  parameter Real integralLimit = 0.2;
  parameter Real outputLimit = 0.35;
  parameter Real derivativeCutoffHz(unit = "Hz") = 25.0;

  Interfaces.FeedbackInput rateAxis
    "Standalone rate-axis channel used for controller qualification";
  Interfaces.PilotInput pilot "Receiver and attitude-estimate samples";
  Interfaces.MixerInput allocation "Throttle and body-rate corrections";

  Interfaces.FeedbackOutput rateAxisResponse;
  Interfaces.PilotCommands pilotCommands;
  Interfaces.MotorCommands motorCommands;
  Interfaces.TelemetrySource telemetry
    "Fixed deployable telemetry interface; compositions may route it onto a bus";

protected
  Control.PidController pid(params = Control.PidParameters(
    samplePeriod = samplePeriod,
    kp = kp,
    ki = ki,
    kd = kd,
    integralLimit = integralLimit,
    commandMin = -outputLimit,
    commandMax = outputLimit,
    derivativeCutoffHz = derivativeCutoffHz));
  CommandMapping pilotMapping(samplePeriod = samplePeriod);
  Mixer mixer(samplePeriod = samplePeriod);

equation
  pid.setpoint = rateAxis.setpoint;
  pid.measurement = rateAxis.measurement;
  rateAxisResponse.error = pid.error;
  rateAxisResponse.command = pid.command;

  pilotMapping.pilot.rcUs = pilot.rcUs;
  pilotMapping.pilot.attitude_rad = pilot.attitude_rad;
  pilotMapping.pilot.throttleForCommand = pilot.throttleForCommand;
  pilotMapping.pilot.armed = pilot.armed;
  pilotCommands.armSwitchHigh = pilotMapping.commands.armSwitchHigh;
  pilotCommands.throttleInput = pilotMapping.commands.throttleInput;
  pilotCommands.throttleCommand = pilotMapping.commands.throttleCommand;
  pilotCommands.acroRateDesired_rad_s =
    pilotMapping.commands.acroRateDesired_rad_s;
  pilotCommands.attitudeDesired_rad = pilotMapping.commands.attitudeDesired_rad;

  mixer.inputSignal.throttle = allocation.throttle;
  mixer.inputSignal.rateCorrection = allocation.rateCorrection;
  motorCommands.motor = mixer.commands.motor;

  telemetry.pidError = rateAxisResponse.error;
  telemetry.pidCommand = rateAxisResponse.command;
  telemetry.throttleCommand = pilotCommands.throttleCommand;
  telemetry.motor = motorCommands.motor;

  annotation(Documentation(info = "<html>
    <p>The public interface mirrors the three causal stages: one PID-axis
    diagnostic, pilot-command mapping, and motor allocation. Vector signals
    remain vectors at the boundary; the implementation introduces no
    scalarized roll/pitch/yaw or motor aliases.</p>
    <p>The fixed telemetry connector is observational only. A composition may
    connect it to an expandable qualification bus without changing the eFMI
    boundary or the controller calculation.</p>
  </html>"));
end Controller;
