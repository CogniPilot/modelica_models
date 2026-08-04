within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block FlightControlSystem
  "Ideal RTOS composition of planning, guidance, rate, and allocation tasks"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Integer maxWaypoints(min = 2) = 8;
  parameter Real planningPeriod(unit = "s") = 0.02;
  parameter Real guidancePeriod(unit = "s") = 0.005;
  parameter Real ratePeriod(unit = "s") = 0.001;

  Planning.Interfaces.WaypointPlanInput plan(capacity = maxWaypoints);
  Interfaces.NavigationEstimateInput navigation;
  input Boolean armed;

  Planning.Interfaces.TrajectoryReferenceOutput reference;
  Interfaces.MotorCommands motorCommands;
  output Real thrust_N;

protected
  Planning.Bezier.WaypointTrajectoryPlanner planningTask(
    maxWaypoints = maxWaypoints,
    samplePeriod = planningPeriod);
  Vehicles.Rdd2.LogLinearController guidanceTask(
    samplePeriod = guidancePeriod);
  Vehicles.Rdd2.RateControlAllocator rateTask(
    samplePeriod = ratePeriod);
  Real headingQuaternionReference[4];

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
  reference.valid = planningTask.reference.valid;
  reference.complete = planningTask.reference.complete;
  reference.sequence = planningTask.reference.sequence;
  reference.activeSegment = planningTask.reference.activeSegment;
  reference.trajectoryTime = planningTask.reference.trajectoryTime;
  reference.totalDuration = planningTask.reference.totalDuration;
  reference.position = planningTask.reference.position;
  reference.velocity = planningTask.reference.velocity;
  reference.acceleration = planningTask.reference.acceleration;
  reference.jerk = planningTask.reference.jerk;
  reference.snap = planningTask.reference.snap;
  reference.yaw = planningTask.reference.yaw;
  reference.yawRate = planningTask.reference.yawRate;
  reference.yawAcceleration = planningTask.reference.yawAcceleration;

  // Navigation and reference routing into the guidance task.
  headingQuaternionReference =
    LieGroups.SO3.EulerB321.to_Quat({reference.yaw, 0.0, 0.0});
  guidanceTask.positionWorld = navigation.positionWorld_m;
  guidanceTask.velocityWorld = navigation.velocityWorld_m_s;
  guidanceTask.quaternionWorldBody = navigation.quaternionWorldBody;
  guidanceTask.positionReferenceWorld = reference.position;
  guidanceTask.velocityReferenceWorld = reference.velocity;
  guidanceTask.accelerationReferenceWorld = reference.acceleration;
  guidanceTask.headingQuaternionReference = headingQuaternionReference;
  guidanceTask.resetIntegral = not armed;

  // Guidance publication into the faster rate-control task.
  rateTask.inputSignal.armed = armed;
  rateTask.inputSignal.thrust_N = guidanceTask.thrust;
  rateTask.inputSignal.angularVelocitySetpointFlu_rad_s =
    guidanceTask.angularVelocitySetpoint;
  rateTask.inputSignal.angularVelocityCorrectionFlu_rad_s =
    guidanceTask.angularVelocityCorrection;
  rateTask.inputSignal.angularVelocityMeasuredFrd_rad_s =
    navigation.angularVelocityBodyFrd_rad_s;
  motorCommands.motor = rateTask.commands.motor;
  thrust_N = guidanceTask.thrust;

  annotation(Documentation(info = "<html>
    <p>This is the executable ideal-RTOS routing model for the deployable RDD2
    functions. Mission plans enter the planning task, trajectory references
    are published to guidance, and guidance commands feed the faster body-rate
    and allocation task. The three periods are explicit parameters so an
    integration test exercises the same multi-rate boundaries intended for
    separate eFMUs.</p>
    <p>The model assumes zero transport delay and deterministic task release at
    phase zero. Concrete RTOS and message transports can be compared against
    this ideal composition without changing the task models themselves.</p>
  </html>"));
end FlightControlSystem;

