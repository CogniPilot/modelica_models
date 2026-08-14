within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block GuidanceController
  "Pilot, attitude, and position guidance task"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.005;
  parameter Real maximumCollectiveThrust_N = 41.3749272
    "Four RDD2 rotors at maximum speed";

  input Integer mode(min = 0, max = 2)
    "0=acro, 1=attitude, 2=position";
  input Boolean armed;
  Interfaces.PilotInput pilot;
  Interfaces.GuidanceStateInput navigation;
  Interfaces.TrajectoryReferenceInput reference;

  Interfaces.PilotCommands pilotCommands;
  Interfaces.RateCommand rateCommand;
  output Real angularVelocitySetpointFlu_rad_s[3];

protected
  CommandMapping pilotMapping(samplePeriod = samplePeriod);
  LogLinearController positionGuidance(samplePeriod = samplePeriod);
  Real navigationEulerB321_rad[3] "{yaw, pitch, roll}";
  Real attitudeReferenceQuaternion[4];
  Real attitudeRateSetpointFlu_rad_s[3];
  Real rateCorrectionFlu_rad_s[3];

equation
  pilotMapping.pilot.stick = pilot.stick;
  pilotMapping.pilot.throttle = pilot.throttle;
  connect(pilotMapping.commands, pilotCommands);

  navigationEulerB321_rad = LieGroups.SO3.EulerB321.from_Quat(
    navigation.quaternionWorldBody);
  attitudeReferenceQuaternion = LieGroups.SO3.EulerB321.to_Quat({
    navigationEulerB321_rad[1],
    pilotCommands.attitudeTiltDesired_rad[2],
    pilotCommands.attitudeTiltDesired_rad[1]});
  attitudeRateSetpointFlu_rad_s =
    Control.Multirotor.LogLinear.attitudeControl(
      positionGuidance.attitudeGain,
      navigation.quaternionWorldBody,
      attitudeReferenceQuaternion);

  positionGuidance.positionWorld = navigation.positionWorldEnu_m;
  positionGuidance.velocityWorld = navigation.velocityWorldEnu_m_s;
  positionGuidance.quaternionWorldBody = navigation.quaternionWorldBody;
  positionGuidance.positionReferenceWorld = reference.positionWorld_m;
  positionGuidance.velocityReferenceWorld = reference.velocityWorld_m_s;
  positionGuidance.accelerationReferenceWorld =
    reference.accelerationWorld_m_s2;
  positionGuidance.headingQuaternionReference =
    LieGroups.SO3.EulerB321.to_Quat({reference.yaw_rad, 0.0, 0.0});
  positionGuidance.resetIntegral = not armed or mode <> 2;

  rateCommand.thrust_N = if mode == 2 then
      positionGuidance.thrust
    else
      maximumCollectiveThrust_N * pilotCommands.throttleInput ^ 2;
  angularVelocitySetpointFlu_rad_s = if mode == 2 then
      positionGuidance.angularVelocitySetpoint
    elseif mode == 1 then
      {1.0, 1.0, 0.0} .* attitudeRateSetpointFlu_rad_s
      + {0.0, 0.0, 1.0} .* pilotCommands.acroRateDesired_rad_s
    else
      pilotCommands.acroRateDesired_rad_s;
  rateCorrectionFlu_rad_s = if mode == 2 then
      positionGuidance.angularVelocityCorrection
    else
      zeros(3);
  rateCommand.angularVelocityCommandFlu_rad_s =
    angularVelocitySetpointFlu_rad_s + rateCorrectionFlu_rad_s;

  annotation(Documentation(info = "<html>
    <p>This block is one deployable RTOS task and one eFMU. It turns pilot,
    navigation, and trajectory messages into the compact rate-command message
    consumed by the fast control task.</p>
    <p>Vector signals stay vector equations throughout the public model.</p>
  </html>"));
end GuidanceController;
