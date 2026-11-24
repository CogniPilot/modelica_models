within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block GuidanceController
  "Pilot, attitude, and position guidance task"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.005;
  parameter Real maximumCollectiveThrust_N = 41.3749272
    "Four RDD2 rotors at maximum speed";

  parameter Real maximumTrajectoryFeedforwardRate_rad_s(min = 0.0,
      unit = "rad/s") = 3.5
    "Norm limit for trajectory rate feed-forward; zero disables it. This
     bounds the flatness singularity near zero thrust, not feedback rates";

  input Integer mode(min = 0, max = 3)
    "0=acro, 1=attitude, 2=mission, 3=pilot position";
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
  Planning.Bezier.MultirotorTrajectory flatTrajectory;
  Planning.Bezier.FlatReference flatReference;
  Real angularVelocityFeedforwardRawFlu_rad_s[3];
  Real angularVelocityFeedforwardNorm_rad_s;
  Real angularVelocityFeedforwardFlu_rad_s[3];
  Boolean useRateFeedforward;

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
  // Mission and pilot position guidance are the same cascade on different
  // reference sources, so both hold the integral and both take the outer-loop
  // thrust and rate command.
  positionGuidance.resetIntegral = not armed or mode < 2;

  // The polynomial planner already publishes jerk and yaw rate. Preserve
  // their angular-rate feed-forward instead of making attitude feedback
  // chase a changing thrust direction from zero rate at every sample.
  flatTrajectory.position = reference.positionWorld_m;
  flatTrajectory.velocity = reference.velocityWorld_m_s;
  // No flatness attitude exists at zero specific thrust. Keep the existing
  // outer-loop fallback there; irrelevant mission inputs must not activate
  // the new reconstruction while flying in either manual attitude mode.
  useRateFeedforward = mode >= 2 and MathUtilities.norm3(
    reference.accelerationWorld_m_s2 + {0.0, 0.0, positionGuidance.gravity})
      > 1.0e-3;
  flatTrajectory.acceleration = if useRateFeedforward then
    reference.accelerationWorld_m_s2 else zeros(3);
  flatTrajectory.jerk = if useRateFeedforward then
    reference.jerkWorld_m_s3 else zeros(3);
  flatTrajectory.snap = zeros(3);
  flatTrajectory.yaw = if useRateFeedforward then reference.yaw_rad else 0.0;
  flatTrajectory.yawRate = if useRateFeedforward then
    reference.yawRate_rad_s else 0.0;
  flatTrajectory.yawAcceleration = 0.0;
  flatReference = Planning.Bezier.flatReference(
    flatTrajectory, positionGuidance.mass, positionGuidance.gravity,
    identity(3));
  // The flatness rate is expressed in the desired body frame. Transport it
  // through world into the measured body frame before adding rate feedback.
  angularVelocityFeedforwardRawFlu_rad_s =
    transpose(LieGroups.SO3.Quat.to_DCM(navigation.quaternionWorldBody))
      * flatReference.bodyToWorld * flatReference.angularVelocityBody;
  // Preserve rate direction while bounding jerk/specific-thrust near the
  // flatness singularity. The default matches the slower pilot yaw envelope.
  // Feedback retains its existing authority and is not clipped here.
  angularVelocityFeedforwardNorm_rad_s =
    MathUtilities.norm3(angularVelocityFeedforwardRawFlu_rad_s);
  angularVelocityFeedforwardFlu_rad_s =
    if angularVelocityFeedforwardNorm_rad_s
        > max(0.0, maximumTrajectoryFeedforwardRate_rad_s) then
      max(0.0, maximumTrajectoryFeedforwardRate_rad_s)
        * angularVelocityFeedforwardRawFlu_rad_s
        / angularVelocityFeedforwardNorm_rad_s
    else angularVelocityFeedforwardRawFlu_rad_s;

  rateCommand.thrust_N = if mode >= 2 then
      positionGuidance.thrust
    else
      maximumCollectiveThrust_N * pilotCommands.throttleInput ^ 2;
  angularVelocitySetpointFlu_rad_s = if mode >= 2 then
      positionGuidance.angularVelocitySetpoint
        + angularVelocityFeedforwardFlu_rad_s
    elseif mode == 1 then
      {1.0, 1.0, 0.0} .* attitudeRateSetpointFlu_rad_s
      + {0.0, 0.0, 1.0} .* pilotCommands.acroRateDesired_rad_s
    else
      pilotCommands.acroRateDesired_rad_s;
  rateCorrectionFlu_rad_s = if mode >= 2 then
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
