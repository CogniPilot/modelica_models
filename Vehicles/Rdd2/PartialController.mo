within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

partial block PartialController
  "Swappable whole-controller boundary for the RDD2 vehicle"
  import Interfaces = Vehicles.Rdd2.ControllerInterfaces;

  parameter Integer maxWaypoints(min = 2) = 16;
  parameter Real planningPeriod(unit = "s") = 0.02;
  parameter Real guidancePeriod(unit = "s") = 0.005;
  parameter Real ratePeriod(unit = "s") = 0.000625;
  parameter Real manualHorizontalSpeed_m_s(unit = "m/s", min = 0.0) = 5.0
    "Reference speed at full horizontal stick in pilot position mode";
  parameter Real manualClimbSpeed_m_s(unit = "m/s", min = 0.0) = 2.0
    "Reference climb rate at full up throttle in pilot position mode";
  parameter Real manualDescentSpeed_m_s(unit = "m/s", min = 0.0) = 1.0
    "Reference sink rate at full down throttle in pilot position mode";
  parameter Real manualHeadingRate_rad_s(unit = "rad/s", min = 0.0) = 1.5
    "Heading rate at full yaw stick in pilot position mode";
  parameter Real manualHorizontalLeash_m(unit = "m", min = 0.0) = 2.0
    "Largest horizontal reference offset the pilot reference may hold";
  parameter Real manualVerticalLeash_m(unit = "m", min = 0.0) = 1.0
    "Largest vertical reference offset the pilot reference may hold";
  parameter Real manualHorizontalSpeedLeash_m_s(unit = "m/s", min = 0.0) = 2.0
    "Largest horizontal speed the pilot reference may lead the vehicle by";
  parameter Real manualVerticalSpeedLeash_m_s(unit = "m/s", min = 0.0) = 1.0
    "Largest vertical speed the pilot reference may lead the vehicle by";

  Planning.Interfaces.WaypointPlanInput plan(capacity = maxWaypoints);
  Avionics.NavigationEstimateInput navigation;
  Interfaces.PilotInput pilot;
  input Integer mode(min = 0, max = 3)
    "0=acro, 1=attitude, 2=mission, 3=pilot position";
  input Boolean armed;

  Planning.Interfaces.TrajectoryReferenceOutput reference;
  Interfaces.MotorCommands motorCommands;
  output Real thrust_N(unit = "N");

  annotation(Documentation(info = "<html>
    <p>This is the controller-side twin of
    <code>Estimation.StrapdownINS.PartialEstimator</code>. Every RDD2 flight
    controller implements this one boundary, so the vehicle system can swap the
    controller the same way it swaps the estimator: one
    <code>redeclare block ControllerModel</code> line changes the controller
    and touches nothing else in the closed loop.</p>
    <p>The boundary carries the navigation estimate, the pilot command, the
    mission plan (one trajectory-reference source), the flight
    <code>mode</code>, and <code>armed</code> on the input side, and publishes
    the accepted trajectory reference, the normalized motor commands, and the
    collective thrust on the output side. Mode 3 selects the other
    trajectory-reference source, the one the pilot pushes with the sticks; its
    speed, rate, and leash limits are airframe configuration and so are
    parameters of this boundary rather than of the block that implements it. All quantities are physical SI in the
    shared ENU/FLU convention and follow the house timestamped-record and
    hold-with-novelty sampling doctrine that the composed tasks already obey.</p>
    <p>The boundary is deliberately whole-controller and RDD2-specific rather
    than a vehicle-independent monolith: the actuator vector width (four
    rotors), the four-mode taxonomy, and the collective-thrust semantics are
    airframe specific, so the honest home is beside the RDD2 controller blocks,
    exactly as the strapdown estimator boundary lives beside its estimator
    family. Finer, vehicle-independent rate, outer, and allocator contracts
    (with a parameterized motor count) are where firmware ports plug in and are
    a separate decomposition layered under this whole-controller boundary; this
    partial does not depend on them, and no allocator-to-controller feedback is
    exposed because the RDD2 clip-then-allocate stage has no such feedback
    path.</p>
  </html>"));
end PartialController;
