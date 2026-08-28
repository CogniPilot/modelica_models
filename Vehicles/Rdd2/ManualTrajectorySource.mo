within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block ManualTrajectorySource
  "Trajectory reference the pilot pushes along SE_2(3) with the sticks"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Real samplePeriod(unit = "s") = 0.01
    "Release interval of the manual guidance task";
  parameter Real stickDeadband(min = 0.0, max = 0.99) = 0.1
    "Normalized dead band on every stick; PX4 MAN_DEADZONE";
  parameter Real stickExpo(min = 0.0, max = 1.0) = 0.6
    "Cubic fraction of the stick curve; the fixed expo in PX4 Sticks.hpp";
  parameter Real horizontalSpeed_m_s(unit = "m/s", min = 0.0) = 5.0
    "Reference speed at full horizontal stick; PX4 MPC_VEL_MANUAL";
  parameter Real climbSpeed_m_s(unit = "m/s", min = 0.0) = 2.0
    "Reference climb rate at full up throttle; PX4 MPC_Z_VEL_MAX_UP";
  parameter Real descentSpeed_m_s(unit = "m/s", min = 0.0) = 1.0
    "Reference sink rate at full down throttle; PX4 MPC_Z_VEL_MAX_DN";
  parameter Real horizontalAcceleration_m_s2(unit = "m/s2", min = 0.0) = 3.0
    "Horizontal acceleration bound of the reference; PX4 MPC_ACC_HOR";
  parameter Real verticalAcceleration_m_s2(unit = "m/s2", min = 0.0) = 3.0
    "Vertical acceleration bound of the reference; PX4 MPC_ACC_UP_MAX";
  parameter Real jerk_m_s3(unit = "m/s3", min = 0.0) = 8.0
    "Jerk bound of the reference; PX4 MPC_JERK_MAX";
  parameter Real headingRate_rad_s(unit = "rad/s", min = 0.0) = 1.5
    "Heading rate at full yaw stick; PX4 MPC_MAN_Y_MAX";
  parameter Real headingRateTimeConstant_s(unit = "s", min = 0.0) = 0.08
    "First-order lag on the heading rate; PX4 MPC_MAN_Y_TAU";
  parameter Real horizontalLeash_m(unit = "m", min = 0.0) = 2.0
    "Bound on the horizontal position block of the pose error log";
  parameter Real verticalLeash_m(unit = "m", min = 0.0) = 1.0
    "Bound on the vertical position block of the pose error log";
  parameter Real horizontalSpeedLeash_m_s(unit = "m/s", min = 0.0) = 2.0
    "Bound on the horizontal velocity block; ArduPilot loiter velocity
     correction limit";
  parameter Real verticalSpeedLeash_m_s(unit = "m/s", min = 0.0) = 1.0
    "Bound on the vertical velocity block of the pose error log";
  parameter Real headingLeash_rad(unit = "rad", min = 0.0) = 0.5235987755982988
    "Bound on the rotation block of the pose error log, 30 degrees";

  input Boolean engaged "The pilot has selected this guidance source";
  Interfaces.PilotInput pilot;
  Interfaces.GuidanceStateInput navigation;

  discrete output Real extendedPose[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true)
    "Reference on SE_2(3): {position ENU, velocity ENU, heading quaternion}";
  discrete output Real positionWorldEnu_m[3](
    each unit = "m", each start = 0.0, each fixed = true);
  discrete output Real velocityWorldEnu_m_s[3](
    each unit = "m/s", each start = 0.0, each fixed = true);
  discrete output Real accelerationWorldEnu_m_s2[3](
    each unit = "m/s2", each start = 0.0, each fixed = true);
  discrete output Real headingQuaternionWorldBody[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete output Real yaw_rad(unit = "rad", start = 0.0, fixed = true)
    "Heading of the reference pose, published for the message schema";
  discrete output Real headingRateCommand_rad_s(
    unit = "rad/s", start = 0.0, fixed = true);
  discrete output Real bodyAcceleration_m_s2[3](
    each unit = "m/s2", each start = 0.0, each fixed = true)
    "Commanded acceleration of the reference in its own heading frame";

protected
  discrete Boolean wasEngaged(start = false, fixed = true);
  discrete Boolean continuing(start = false, fixed = true)
    "This sample continues an engagement rather than beginning one";
  discrete Real vehicleEulerB321_rad[3](each start = 0.0, each fixed = true)
    "{yaw, pitch, roll} of the navigation estimate";
  discrete Real referenceEulerB321_rad[3](
    each start = 0.0, each fixed = true);
  discrete Real vehicleHeadingQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real vehiclePose[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true)
    "Estimate as an SE_2(3) element with the attitude projected to heading";
  discrete Real basePose[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true);
  discrete Real baseAcceleration[3](each start = 0.0, each fixed = true);
  discrete Real baseHeadingRate(start = 0.0, fixed = true);
  discrete Real baseBodyVelocity[3](each start = 0.0, each fixed = true)
    "Reference velocity carried into the reference heading frame";
  discrete Real shapedRoll(start = 0.0, fixed = true);
  discrete Real shapedPitch(start = 0.0, fixed = true);
  discrete Real shapedYaw(start = 0.0, fixed = true);
  discrete Real shapedThrottle(start = 0.0, fixed = true);
  discrete Real verticalSpeedScale(start = 0.0, fixed = true);
  discrete Real stickDisc[2](each start = 0.0, each fixed = true)
    "{forward, left} stick command of unit length or less";
  discrete Real targetBodyVelocity[3](each start = 0.0, each fixed = true);
  discrete Real velocityError[3](each start = 0.0, each fixed = true);
  discrete Real horizontalRateTarget[2](each start = 0.0, each fixed = true);
  discrete Real verticalRateTarget(start = 0.0, fixed = true);
  discrete Real horizontalRateStep[2](each start = 0.0, each fixed = true);
  discrete Real verticalRateStep(start = 0.0, fixed = true);
  discrete Real commandAcceleration[3](each start = 0.0, each fixed = true);
  discrete Real commandHeadingRate(start = 0.0, fixed = true);
  discrete Real stepBodyIncrement[9](each start = 0.0, each fixed = true)
    "Body-frame se_2(3) increment of one sample, ordered
     {position, velocity, rotation}";
  discrete Real unleashedPose[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true);
  discrete Real poseError[9](each start = 0.0, each fixed = true)
    "log(vehiclePose^-1 * referencePose), the left-invariant error";
  discrete Real leashedError[9](each start = 0.0, each fixed = true);
  discrete Real leashedPoseRaw[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true);
  discrete Real leashedPose[10](
    start = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
    each fixed = true);

equation
  assert(samplePeriod > 0.0,
    "ManualTrajectorySource sample period must be positive");
  assert(horizontalLeash_m > 0.0 and verticalLeash_m > 0.0
    and horizontalSpeedLeash_m_s > 0.0 and verticalSpeedLeash_m_s > 0.0
    and headingLeash_rad > 0.0,
    "ManualTrajectorySource leash bounds must be positive");
  assert(headingLeash_rad < 3.141592653589793,
    "ManualTrajectorySource heading leash must stay inside the principal
     logarithm domain");

algorithm
  when sample(0.0, samplePeriod) then
    // Project the navigation estimate onto the level extended pose: position
    // and velocity in R^3 and heading on the circle. This is the one chart in
    // the block, taken at the boundary where a full attitude enters, and it
    // uses the repo's single heading convention so the stick frame, the leash
    // frame, and the guidance attitude reference all mean the same heading.
    vehicleEulerB321_rad := LieGroups.SO3.EulerB321.from_Quat(
      navigation.quaternionWorldBody);
    vehicleHeadingQuaternion := LieGroups.SO3.EulerB321.to_Quat({
      vehicleEulerB321_rad[1], 0.0, 0.0});
    vehiclePose := cat(1,
      navigation.positionWorldEnu_m,
      navigation.velocityWorldEnu_m_s,
      vehicleHeadingQuaternion);

    // Bumpless entry is one assignment. While this source is not selected the
    // reference IS the estimated extended pose, so on the entry sample the
    // reference already carries the vehicle's position, its velocity, and its
    // heading together. Entering at speed needs no separate velocity seeding
    // because the velocity is part of the group element.
    continuing := engaged and pre(wasEngaged);
    basePose := if continuing then pre(extendedPose) else vehiclePose;
    baseAcceleration := if continuing then pre(bodyAcceleration_m_s2)
      else zeros(3);
    baseHeadingRate := if continuing then pre(headingRateCommand_rad_s)
      else 0.0;
    baseBodyVelocity := LieGroups.SO3.Quat.rotate(
      LieGroups.SO3.Quat.inverse(basePose[7:10]), basePose[4:6]);

    // Stick shaping. Dead band first, then expo, so the curve is continuous
    // at the band edge and still reaches full authority at full stick.
    shapedRoll := MathUtilities.expo(
      MathUtilities.deadzone(pilot.stick[1], stickDeadband), stickExpo);
    shapedPitch := MathUtilities.expo(
      MathUtilities.deadzone(pilot.stick[2], stickDeadband), stickExpo);
    shapedYaw := MathUtilities.expo(
      MathUtilities.deadzone(pilot.stick[3], stickDeadband), stickExpo);
    shapedThrottle := MathUtilities.expo(
      MathUtilities.deadzone(
        2.0 * MathUtilities.clip(pilot.throttle, 0.0, 1.0) - 1.0,
        stickDeadband),
      stickExpo);
    // The square stick gate is limited to the unit disc so a diagonal command
    // is not faster than an axis command.
    stickDisc := MathUtilities.limitNorm({shapedPitch, -shapedRoll}, 1.0);
    verticalSpeedScale := if shapedThrottle >= 0.0 then climbSpeed_m_s
      else descentSpeed_m_s;

    // The sticks command a velocity in the reference's own frame. No rotation
    // appears here: the group composition below carries the command into the
    // world frame, which is what makes the mapping heading-relative by
    // construction rather than by an explicit yaw rotation.
    targetBodyVelocity := {
      horizontalSpeed_m_s * stickDisc[1],
      horizontalSpeed_m_s * stickDisc[2],
      shapedThrottle * verticalSpeedScale};

    // Acceleration that closes the velocity error without overshooting once
    // the jerk bound has to unwind the command.
    velocityError := targetBodyVelocity - baseBodyVelocity;
    horizontalRateTarget := MathUtilities.limitNorm(
      velocityError[1:2] / samplePeriod,
      min(horizontalAcceleration_m_s2,
        MathUtilities.squareRootRamp(
          MathUtilities.norm2(velocityError[1:2]),
          1.0 / samplePeriod,
          jerk_m_s3)));
    verticalRateTarget := MathUtilities.clip(
      MathUtilities.squareRootRamp(
        velocityError[3], 1.0 / samplePeriod, jerk_m_s3),
      -verticalAcceleration_m_s2, verticalAcceleration_m_s2);
    // Hard jerk guard. The ramp above already rides the jerk bound while it
    // decelerates; this bounds the step a slammed stick can command.
    horizontalRateStep := MathUtilities.limitNorm(
      horizontalRateTarget - baseAcceleration[1:2], jerk_m_s3 * samplePeriod);
    verticalRateStep := MathUtilities.clip(
      verticalRateTarget - baseAcceleration[3],
      -jerk_m_s3 * samplePeriod, jerk_m_s3 * samplePeriod);
    commandAcceleration := cat(1,
      baseAcceleration[1:2] + horizontalRateStep,
      {baseAcceleration[3] + verticalRateStep});
    commandHeadingRate := baseHeadingRate
      + samplePeriod / (headingRateTimeConstant_s + samplePeriod)
        * (headingRate_rad_s * shapedYaw - baseHeadingRate);

    // One sample of motion is one exact flow. The body increment carries the
    // commanded acceleration and heading rate, the world increment is zero
    // because a reference is not in free fall, and the nilpotent time block
    // carries the velocity-to-position coupling exactly, so the double
    // integration a pushed reference needs is one call rather than two
    // hand-written integrators. This is the same primitive and the same
    // argument order that Estimation.StrapdownINS.ESKF.predictNominal uses to
    // propagate the nominal state, with gravity set to zero.
    stepBodyIncrement := cat(1,
      zeros(3),
      commandAcceleration * samplePeriod,
      {0.0, 0.0, commandHeadingRate * samplePeriod});
    unleashedPose := LieGroups.SE23.Quat.exp_mixed(
      basePose,
      stepBodyIncrement,
      zeros(9),
      [0.0, samplePeriod; 0.0, 0.0]);

    // Leash. The bound is applied to the logarithm of the relative transform,
    // so it is a geodesic projection of the error rather than a clamp of world
    // coordinates, and the reference is then re-placed by the exponential of
    // the clamped error from the vehicle's own extended pose. The position
    // block keeps the reference from running away from a vehicle held back by
    // wind, a thrust limit, or a saturated tilt; the velocity block keeps its
    // speed from running away from the same vehicle, which is the anti-windup
    // of the velocity feed-forward and is only expressible because velocity
    // lives inside the group element.
    poseError := LieGroups.SE23.Quat.log_map(
      LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(vehiclePose), unleashedPose));
    leashedError := cat(1,
      MathUtilities.limitNorm(poseError[1:2], horizontalLeash_m),
      {MathUtilities.clip(poseError[3], -verticalLeash_m, verticalLeash_m)},
      MathUtilities.limitNorm(poseError[4:5], horizontalSpeedLeash_m_s),
      {MathUtilities.clip(
         poseError[6], -verticalSpeedLeash_m_s, verticalSpeedLeash_m_s),
       0.0,
       0.0,
       MathUtilities.clip(
         poseError[9], -headingLeash_rad, headingLeash_rad)});
    leashedPoseRaw := LieGroups.SE23.Quat.product(
      vehiclePose, LieGroups.SE23.Quat.exp_map(leashedError));
    leashedPose := cat(1,
      leashedPoseRaw[1:6],
      LieGroups.SO3.Quat.normalize(leashedPoseRaw[7:10]));

    // Selecting the accepted reference is an expression rather than a guarded
    // assignment, so every state of this task is written on exactly one
    // unconditional path per sample.
    extendedPose := if engaged then leashedPose else vehiclePose;
    bodyAcceleration_m_s2 := if engaged then commandAcceleration else zeros(3);
    headingRateCommand_rad_s := if engaged then commandHeadingRate else 0.0;

    // Publication into the fixed-layout trajectory message. Position and
    // velocity are group coordinates and need no conversion. The SE_2(3)
    // velocity block is a world-frame translation, so its derivative is the
    // rotated body acceleration with no transport term.
    positionWorldEnu_m := extendedPose[1:3];
    velocityWorldEnu_m_s := extendedPose[4:6];
    headingQuaternionWorldBody := extendedPose[7:10];
    accelerationWorldEnu_m_s2 := LieGroups.SO3.Quat.rotate(
      headingQuaternionWorldBody, bodyAcceleration_m_s2);
    referenceEulerB321_rad := LieGroups.SO3.EulerB321.from_Quat(
      headingQuaternionWorldBody);
    yaw_rad := referenceEulerB321_rad[1];

    wasEngaged := engaged;
  end when;

  annotation(Documentation(info = "<html>
    <h4>What the sticks command</h4>
    <p>The sticks command the motion of the trajectory reference, not of the
    vehicle. The reference is an extended pose that integrates the shaped
    command and the existing position cascade tracks it, so this block adds a
    setpoint source and no second controller.</p>
    <h4>The group, and why this one</h4>
    <p>The reference is an element of SE_2(3),
    <code>{position ENU, velocity ENU, heading quaternion}</code>, the same
    group and the same coordinate layout the tracking loop and the estimator
    already use. A carrot has intrinsic velocity state, because its command is
    acceleration limited and its velocity is the feed-forward the cascade
    consumes, so velocity belongs inside the group element rather than beside
    it. The reference then maps one to one onto the four reference inputs of
    <code>Control.Multirotor.LogLinear.Controller</code>: position, velocity,
    the heading quaternion, and the rotated body acceleration.</p>
    <p>Four things follow from the group formulation rather than being coded.
    The heading-frame stick mapping is just the fact that the command lives in
    the reference's own frame. One sample of the carrot's double integration,
    stick acceleration to velocity to position while the heading turns, is one
    exponential of one tangent vector. Bumpless entry is one assignment of the
    estimated extended pose, position, velocity, and heading together. And the
    update is left equivariant, so the same stick sequence flown from a rotated
    and translated initial pose produces the rotated and translated trajectory
    exactly.</p>
    <p>The rotation block is generated only by the world-vertical generator, so
    the reference quaternion is a pure heading rotation for all time and
    heading wrap-around is not representable rather than handled. There is no
    angle accumulation and no angle normalization in this block. The pilot
    commands no roll or pitch in this mode, which is why the reference attitude
    the guidance already expects is heading only.</p>
    <h4>One flow, shared with the estimator</h4>
    <p>A sample of reference motion is one call to
    <code>LieGroups.SE23.Quat.exp_mixed</code>, the exact flow of the
    time-extended SE_2(3) in which a world-frame term is a left factor, the
    body input a right factor, and the velocity-to-position coupling exact
    through a nilpotent time block. It is the same primitive, with the same
    argument order, that
    <code>Estimation.StrapdownINS.ESKF.predictNominal</code> propagates the
    nominal state with; the only difference is that a reference is not in free
    fall, so the world-frame increment is zero where the estimator passes
    gravity. The commanded acceleration and heading rate are the body
    increment, and the double integration a pushed reference needs falls out
    of the coupling block instead of being written by hand.</p>
    <h4>Error convention</h4>
    <p>The error is <code>log(vehiclePose^-1 * referencePose)</code> on
    SE_2(3), a left-invariant error read in the local frame, and the reference
    advances by right composition <code>X * exp(xi)</code>. This is the same
    convention as both neighbours:
    <code>Control.Multirotor.LogLinear.stateError</code> takes
    <code>log(actual^-1 * reference)</code> through
    <code>LieGroups.SE23.Quat</code>, and
    <code>Estimation.StrapdownINS.ESKF.inject</code> right-injects its local
    tangent correction as <code>X * exp(delta)</code>. Guidance, estimation,
    and this source therefore all read their tangents in the local frame, which
    is the mismatch that would otherwise stay invisible until it flew.</p>
    <h4>Why the reference is pushed rather than latched</h4>
    <p>The alternative, and what PX4's Position mode does, is to control
    velocity while the sticks are deflected and latch a position setpoint once
    the vehicle has stopped. That design carries a discrete lock state, a
    release detector, a braking phase, and a handoff whose two sides have to be
    made to agree. Pushing the reference has none of those: releasing the
    sticks is not a mode change, it is a zero commanded velocity, and the same
    equations decelerate the reference and then hold it. Position control and
    therefore wind rejection are active the whole time, and there is one fewer
    discrete state to reason about. This is ArduPilot's Loiter lineage, where
    the hold target is the running integral of the feed-forward velocity and
    stopping is the degenerate case of the same code path.</p>
    <h4>The three mechanisms a pushed reference needs</h4>
    <ul>
      <li>A leash on the error logarithm. Its position block bounds how far the
      reference may lead a vehicle that cannot keep up, so a release never
      commands a flight back to wherever the reference reached. Its velocity
      block bounds how much faster the reference may be travelling than the
      vehicle, which is the anti-windup of the velocity feed-forward.</li>
      <li>A shaped command: dead band and expo on the sticks, then an
      acceleration and jerk bounded approach to the commanded velocity, so the
      reference decelerates smoothly to rest on release instead of needing an
      explicit braking phase.</li>
      <li>Bumpless entry, which the anchoring above gives without a branch.</li>
    </ul>
    <h4>Known limits</h4>
    <p>Yawing while flying rotates the commanded direction with the heading, so
    the reference follows a curve; the acceleration bound is what limits how
    tight that curve can be, exactly as it limits a straight-line change of
    speed. A yaw rate whose implied centripetal acceleration exceeds the bound
    widens the turn rather than being refused.</p>
    <p><code>Planning.Interfaces.TrajectoryReferenceOutput</code> carries the
    heading as a scalar, so the reference quaternion is also published as
    <code>yaw_rad</code> and rebuilt by the guidance task. The projection is
    exact for a pure heading rotation and no angle is accumulated across
    samples, but carrying the quaternion in the message would remove the chart
    from the path entirely.</p>
    <h4>Conventions</h4>
    <p>Positions and velocities are world ENU. <code>pilot.stick</code> is
    normalized {roll, pitch, yaw} in body FLU, matching
    <code>Vehicles.Rdd2.CommandMapping</code>; <code>pilot.throttle</code> is
    normalized collective whose center commands level flight in this mode. The
    SE_2(3) tangent is ordered {position, velocity, rotation}, as in
    <code>LieGroups.SE23.Quat</code>.</p>
    <h4>Timing</h4>
    <p>One <code>sample</code> clock, every history reference an explicit
    <code>pre</code>. The block is a pure function of this sample's pilot and
    navigation inputs and the previous accepted reference.</p>
  </html>"));
end ManualTrajectorySource;
