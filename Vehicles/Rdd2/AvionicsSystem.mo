within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block AvionicsSystem
  "Ideal RTOS composition of planning and the shared flight controller"
  extends Vehicles.Rdd2.PartialController;
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

protected
  Planning.Bezier.WaypointTrajectoryPlanner planningTask(
    maxWaypoints = maxWaypoints,
    samplePeriod = planningPeriod);
  Vehicles.Rdd2.GuidanceController guidanceTask(
    samplePeriod = guidancePeriod);
  Vehicles.Rdd2.RateControlAllocator rateTask(
    samplePeriod = ratePeriod);
  // Both trajectory sources are the same deployable task on the same clock,
  // so the published reference has one cadence whatever the mode is and the
  // selection below never has to reconcile two message rates.
  Vehicles.Rdd2.ManualTrajectorySource manualTask(
    samplePeriod = planningPeriod,
    horizontalSpeed_m_s = manualHorizontalSpeed_m_s,
    climbSpeed_m_s = manualClimbSpeed_m_s,
    descentSpeed_m_s = manualDescentSpeed_m_s,
    headingRate_rad_s = manualHeadingRate_rad_s,
    horizontalLeash_m = manualHorizontalLeash_m,
    verticalLeash_m = manualVerticalLeash_m,
    horizontalSpeedLeash_m_s = manualHorizontalSpeedLeash_m_s,
    verticalSpeedLeash_m_s = manualVerticalSpeedLeash_m_s);
  Boolean manualEngaged "The pilot has selected manual position guidance";

equation
  // Mission-data transport into the planning task.
  planningTask.plan.valid = plan.valid;
  planningTask.plan.sequence = plan.sequence;
  planningTask.plan.waypointCount = plan.waypointCount;
  planningTask.plan.globalFrame = plan.globalFrame;
  planningTask.plan.originGeodetic = plan.originGeodetic;
  planningTask.plan.waypoint = plan.waypoint;
  planningTask.plan.velocityEnu = plan.velocityEnu;
  planningTask.plan.yaw = plan.yaw;
  planningTask.plan.nominalSpeed = plan.nominalSpeed;
  planningTask.plan.minSegmentDuration = plan.minSegmentDuration;
  // The mission clock stops while the pilot flies, so handing control back
  // resumes the trajectory where it was paused.
  planningTask.hold = manualEngaged;

  // Pilot-pushed trajectory source. It tracks the estimate whenever it is not
  // selected, which is what makes entering the mode bumpless without an entry
  // branch.
  manualEngaged = mode == 3;
  manualTask.engaged = manualEngaged;
  connect(pilot, manualTask.pilot);
  manualTask.navigation.positionWorldEnu_m = navigation.positionWorldEnu_m;
  manualTask.navigation.velocityWorldEnu_m_s =
    navigation.velocityWorldEnu_m_s;
  manualTask.navigation.quaternionWorldBody = navigation.quaternionWorldBody;

  // Published trajectory-reference message. The mode selects which guidance
  // source owns it; the flight controller downstream sees one reference and
  // needs no knowledge of where it came from.
  reference.valid =
    if manualEngaged then true else planningTask.reference.valid;
  reference.complete =
    if manualEngaged then false else planningTask.reference.complete;
  reference.sequence = planningTask.reference.sequence;
  reference.activeSegment = planningTask.reference.activeSegment;
  reference.trajectoryTime = planningTask.reference.trajectoryTime;
  reference.totalDuration = planningTask.reference.totalDuration;
  reference.position = if manualEngaged then manualTask.positionWorldEnu_m
    else planningTask.reference.position;
  reference.velocity = if manualEngaged then manualTask.velocityWorldEnu_m_s
    else planningTask.reference.velocity;
  reference.acceleration =
    if manualEngaged then manualTask.accelerationWorldEnu_m_s2
    else planningTask.reference.acceleration;
  reference.jerk = if manualEngaged then zeros(3)
    else planningTask.reference.jerk;
  reference.snap = if manualEngaged then zeros(3)
    else planningTask.reference.snap;
  reference.yaw = if manualEngaged then manualTask.yaw_rad
    else planningTask.reference.yaw;
  reference.yawRate = if manualEngaged then manualTask.headingRateCommand_rad_s
    else planningTask.reference.yawRate;
  reference.yawAcceleration = if manualEngaged then 0.0
    else planningTask.reference.yawAcceleration;

  // Guidance publishes one compact rate command across the thread boundary.
  guidanceTask.mode = mode;
  guidanceTask.armed = armed;
  connect(pilot, guidanceTask.pilot);
  guidanceTask.navigation.positionWorldEnu_m = navigation.positionWorldEnu_m;
  guidanceTask.navigation.velocityWorldEnu_m_s = navigation.velocityWorldEnu_m_s;
  guidanceTask.navigation.quaternionWorldBody = navigation.quaternionWorldBody;
  guidanceTask.reference.positionWorld_m = reference.position;
  guidanceTask.reference.velocityWorld_m_s = reference.velocity;
  guidanceTask.reference.accelerationWorld_m_s2 = reference.acceleration;
  guidanceTask.reference.yaw_rad = reference.yaw;

  // Rate control shares the fast device thread with IMU input and motor I/O.
  rateTask.inputSignal.armed = armed;
  rateTask.inputSignal.thrust_N = guidanceTask.rateCommand.thrust_N;
  rateTask.inputSignal.angularVelocityCommandFlu_rad_s =
    guidanceTask.rateCommand.angularVelocityCommandFlu_rad_s;
  rateTask.inputSignal.angularVelocityMeasuredFlu_rad_s =
    navigation.angularVelocityBodyFlu_rad_s;
  motorCommands.motor = rateTask.commands.motor;
  thrust_N = guidanceTask.rateCommand.thrust_N;

  annotation(Documentation(info = "<html>
    <p>This is the executable ideal-RTOS routing model for the deployable RDD2
    functions. Planning, guidance, and rate control are separate objects and
    separate deployable eFMUs. Their vector connectors are the readable source
    of the generated fixed-layout message bindings.</p>
    <p>The model assumes zero transport delay and deterministic task release at
    phase zero. Concrete RTOS and message transports can be compared against
    this ideal composition without changing the task models themselves.</p>
    <p>It implements the swappable <code>Vehicles.Rdd2.PartialController</code>
    boundary and is the default flight controller selected by
    <code>Vehicles.Rdd2.WaypointVehicleSystem</code>.</p>
    <p>Two guidance sources publish into the one trajectory-reference message.
    Mode 2 takes the planned mission trajectory; mode 3 takes the reference the
    pilot pushes with the sticks. The selection is the only place the two meet,
    and the position cascade below it is shared rather than duplicated, so a
    pilot-flown hold and a mission leg are tracked by the same controller with
    the same gains and the same integral.</p>
  </html>"));
end AvionicsSystem;
