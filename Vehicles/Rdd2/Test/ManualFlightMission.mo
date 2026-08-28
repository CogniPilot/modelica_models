within Vehicles.Rdd2.Test;

model ManualFlightMission
  "GPS-navigated flight handed from the mission to the pilot and back"
  parameter Real cruiseAltitude_m(unit = "m") = 2.0;
  parameter Real legLength_m(unit = "m") = 4.0;
  parameter Real manualEntry_s(unit = "s") = 5.0
    "Mode switch to pilot position, part way along a moving mission leg";
  parameter Real stickPush_s(unit = "s") = 9.0;
  parameter Real stickRelease_s(unit = "s") = 11.0;
  parameter Real missionResume_s(unit = "s") = 19.0;
  parameter Real settling_s(unit = "s") = 5.0
    "Time allowed for the vehicle to converge on a stopped reference";
  final parameter Real modeAcceptance_s(unit = "s") = 0.3
    "Debounce of the mode switch plus a margin";

  extends Vehicles.Rdd2.WaypointVehicleSystem(
    maxWaypoints = 4,
    waypointCount = 4,
    useGlobalWaypoints = false,
    navigationSource = 1,
    nominalSpeed = 1.0,
    armTime_s = 1.0,
    disarmDelay_s = 30.0,
    manualHorizontalSpeed_m_s = 2.0,
    manualClimbSpeed_m_s = 1.0,
    manualDescentSpeed_m_s = 1.0,
    manualHorizontalLeash_m = 2.0,
    estimatorInitialPositionWorldEnu_m = {0.08, -0.06, 0.04},
    localRoute = [
      0.0,        0.0, 0.0;
      0.0,        0.0, cruiseAltitude_m;
      legLength_m, 0.0, cruiseAltitude_m;
      legLength_m, legLength_m, cruiseAltitude_m],
    transmitterEventCount = 5,
    transmitterEventTime_s = {
      0.0, manualEntry_s, stickPush_s, stickRelease_s, missionResume_s},
    transmitterChannel_us = [
      1500.0, 1500.0, 1500.0, 1500.0, 1900.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1500.0;
      1500.0, 2000.0, 1500.0, 1500.0, 1500.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1500.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1900.0]);

  output Real referenceOffset_m
    "Distance from the published reference to the navigation estimate";
  output Real referenceSpeed_m_s;
  output Real groundSpeed_m_s;
  output Real holdDrift_m
    "Distance from the vehicle to the reference it is holding";
  output Real tiltAngle_rad "Angle between body up and world up";

protected
  Real bodyUpWorld[3];

equation
  referenceOffset_m = MathUtilities.norm3(
    avionics.reference.position - avionics.navigation.positionWorldEnu_m);
  referenceSpeed_m_s = MathUtilities.norm3(avionics.reference.velocity);
  groundSpeed_m_s = MathUtilities.norm3(plant.truth.velocityWorldEnu_m_s);
  holdDrift_m = MathUtilities.norm3(
    plant.truth.positionWorldEnu_m - avionics.reference.position);
  bodyUpWorld = LieGroups.SO3.Quat.rotate(
    plant.truth.quaternionWorldBody, {0.0, 0.0, 1.0});
  tiltAngle_rad = acos(MathUtilities.clip(bodyUpWorld[3], -1.0, 1.0));

  // MODE SEQUENCE. Mission, then pilot position, then mission again, each
  // accepted only after the switch debounce.
  assert(time < 0.05 or time >= manualEntry_s or flightMode == 2,
    "Manual flight mission did not start in mission mode");
  assert(time < manualEntry_s + modeAcceptance_s or time >= missionResume_s
    or flightMode == 3,
    "Mode switch did not hand the vehicle to the pilot");
  assert(time < missionResume_s + modeAcceptance_s or flightMode == 2,
    "Mode switch did not hand the vehicle back to the mission");

  // BUMPLESS ENTRY. The pilot reference is anchored on the estimate at the
  // entry sample, so the setpoint the position loop sees does not step. The
  // vehicle is still flying the mission leg at this instant, so this also
  // witnesses entry at speed: a reference seeded at rest would have to be
  // chased from the vehicle it was left behind by.
  assert(time < manualEntry_s + modeAcceptance_s
    or time >= manualEntry_s + modeAcceptance_s + 0.2
    or referenceOffset_m < 0.3,
    "Pilot reference jumped away from the vehicle at mode entry");

  // LEASH. Whatever the pilot does, the tracking error the position loop is
  // shown stays bounded, so a release never commands a flight back to
  // wherever the reference reached. The margin covers the reference being
  // held between its samples while the vehicle keeps moving.
  assert(time < manualEntry_s + modeAcceptance_s or time >= missionResume_s
    or referenceOffset_m < manualHorizontalLeash_m + 0.2,
    "Pilot reference left the leash, so the setpoint the position loop sees is unbounded");

  // BRAKING AND HOLD. Releasing the sticks is a zero twist, not a mode change;
  // the reference decelerates to rest and the position loop then holds it.
  assert(time < stickRelease_s + settling_s or time >= missionResume_s
    or referenceSpeed_m_s < 0.05,
    "Pilot reference did not come to rest after the sticks were released");
  assert(time < stickRelease_s + settling_s or time >= missionResume_s
    or groundSpeed_m_s < 0.4,
    "Vehicle did not brake to a hover after the sticks were released");
  // The reference is provably at rest by the previous claim, so a bounded
  // distance from it is a bounded distance from a fixed point in the world.
  assert(time < stickRelease_s + settling_s or time >= missionResume_s
    or holdDrift_m < 0.6,
    "Vehicle did not hold station on the reference it had stopped at");

  // NOTHING IN THE HANDOVERS MAY EXCITE THE AIRFRAME. A setpoint step at a
  // mode change would show here as a tilt spike long before it showed as a
  // position error.
  assert(time < armTime_s + 2.0 or tiltAngle_rad < 0.7,
    "Vehicle tilt exceeded 40 degrees, so a mode change or a reference step excited the airframe");

  annotation(
    experiment(
      StartTime = 0.0,
      StopTime = 28.0,
      Tolerance = 1.0e-8,
      Interval = 0.005),
    Documentation(info = "<html>
      <p>One flight through every transition the pilot position mode has: a
      mission leg, a switch to pilot position part way along it while the
      vehicle is moving, a stick push and release, and a switch back to the
      mission. Guidance flies on the GPS-aided inertial solution, so the
      reference is anchored on the estimate rather than on plant truth, which
      is what a real handover has available.</p>
      <p>The mission clock stops while the pilot flies, so the trajectory the
      vehicle returns to is the one it was paused at rather than a point the
      mission ran on to unattended.</p>
      <p>Assertions live in the equation section. An assertion inside a
      when-clause is logged and then ignored by the OpenModelica runtime, and
      this model is simulated by Rumoca in CI, where an equation-section
      assertion ends the run with a non-zero status.</p>
    </html>"));
end ManualFlightMission;
