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

  // Published trajectory-reference message.
  connect(planningTask.reference, reference);

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
  </html>"));
end AvionicsSystem;
