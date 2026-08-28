within Tests;

model ManualGuidanceDynamics
  "Time-domain behaviour of the manual guidance source and the mode switch"

  parameter Real samplePeriod(unit = "s") = 0.01;
  parameter Real horizontalLeash_m(unit = "m") = 2.0;
  parameter Real horizontalSpeedLeash_m_s(unit = "m/s") = 2.0;
  parameter Real cruiseSpeed_m_s(unit = "m/s") = 3.0
    "Speed the third vehicle is already flying when the mode is selected";
  parameter Real relativeTranslation_m[3] = {10.0, -4.0, 1.0}
    "Translation of the equivariance transform";
  parameter Real relativeHeading_rad(unit = "rad") = 0.9
    "Heading of the equivariance transform";
  final parameter Real relativePose[10] = cat(1,
    relativeTranslation_m,
    zeros(3),
    LieGroups.SO3.EulerB321.to_Quat({relativeHeading_rad, 0.0, 0.0}))
    "The transform G relating the two stationary vehicles. Its velocity block
     is zero because a rigid transform of the world does not add speed.";
  Vehicles.Rdd2.ScriptedTransmitter transmitter(
    eventCount = 9,
    eventTime_s = {0.0, 3.0, 4.0, 4.1, 5.0, 6.0, 6.5, 7.0, 8.0},
    channelPwm_us = [
      1500.0, 2000.0, 1500.0, 1500.0, 1900.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1900.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1500.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1900.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1500.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1190.0;
      1500.0, 1500.0, 1500.0, 1500.0,  700.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1170.0;
      1500.0, 1500.0, 1500.0, 1500.0, 1900.0]);
  Vehicles.Rdd2.FlightModeSelector selector(samplePeriod = samplePeriod);
  Vehicles.Rdd2.ManualTrajectorySource held(
    samplePeriod = samplePeriod,
    horizontalLeash_m = horizontalLeash_m,
    horizontalSpeedLeash_m_s = horizontalSpeedLeash_m_s)
    "Flown against a vehicle that cannot move at all";
  Vehicles.Rdd2.ManualTrajectorySource transformed(
    samplePeriod = samplePeriod,
    horizontalLeash_m = horizontalLeash_m,
    horizontalSpeedLeash_m_s = horizontalSpeedLeash_m_s)
    "The same sticks flown against the transformed vehicle";
  Vehicles.Rdd2.ManualTrajectorySource entered(
    samplePeriod = samplePeriod,
    horizontalLeash_m = horizontalLeash_m,
    horizontalSpeedLeash_m_s = horizontalSpeedLeash_m_s)
    "Selected part way through a cruise, to witness bumpless entry";

  Real stickCommand[3];
  Real throttleCommand;
  Real heldOffset_m "Horizontal reference offset from the held vehicle";
  Real heldSpeed_m_s "Horizontal reference speed of the held vehicle's source";
  Real equivarianceResidual
    "Norm of log((G * heldPose)^-1 * transformedPose)";
  Real enteredPositionError_m;
  Real enteredSpeedError_m_s;
  Real enteredOffset_m;

protected
  Real cruisePosition_m[3];
  Real expectedTransformedPose[10];

equation
  stickCommand = {
    Vehicles.Interfaces.centeredPwmToUnit(transmitter.channelPwm_us_out[1]),
    Vehicles.Interfaces.centeredPwmToUnit(transmitter.channelPwm_us_out[2]),
    Vehicles.Interfaces.centeredPwmToUnit(transmitter.channelPwm_us_out[3])};
  throttleCommand =
    Vehicles.Interfaces.throttlePwmToUnit(transmitter.channelPwm_us_out[4]);
  selector.switchPwm_us = transmitter.channelPwm_us_out[5];

  held.engaged = true;
  held.pilot.stick = stickCommand;
  held.pilot.throttle = throttleCommand;
  held.navigation.positionWorldEnu_m = zeros(3);
  held.navigation.velocityWorldEnu_m_s = zeros(3);
  held.navigation.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};

  transformed.engaged = true;
  transformed.pilot.stick = stickCommand;
  transformed.pilot.throttle = throttleCommand;
  transformed.navigation.positionWorldEnu_m = relativePose[1:3];
  transformed.navigation.velocityWorldEnu_m_s = zeros(3);
  transformed.navigation.quaternionWorldBody = relativePose[7:10];

  cruisePosition_m = {cruiseSpeed_m_s * time, 0.0, 2.0};
  entered.engaged = time >= 5.0;
  entered.pilot.stick = zeros(3);
  entered.pilot.throttle = 0.5;
  entered.navigation.positionWorldEnu_m = cruisePosition_m;
  entered.navigation.velocityWorldEnu_m_s = {cruiseSpeed_m_s, 0.0, 0.0};
  entered.navigation.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};

  heldOffset_m = MathUtilities.norm2(held.positionWorldEnu_m[1:2]);
  heldSpeed_m_s = MathUtilities.norm2(held.velocityWorldEnu_m_s[1:2]);
  expectedTransformedPose =
    LieGroups.SE23.Quat.product(relativePose, held.extendedPose);
  equivarianceResidual = sqrt(
    LieGroups.SE23.Quat.log_map(
      LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(expectedTransformedPose),
        transformed.extendedPose))
    * LieGroups.SE23.Quat.log_map(
      LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(expectedTransformedPose),
        transformed.extendedPose)));
  enteredPositionError_m = MathUtilities.norm3(
    entered.positionWorldEnu_m - cruisePosition_m);
  enteredSpeedError_m_s = MathUtilities.norm3(
    entered.velocityWorldEnu_m_s - {cruiseSpeed_m_s, 0.0, 0.0});
  enteredOffset_m = MathUtilities.norm2(
    entered.positionWorldEnu_m[1:2] - cruisePosition_m[1:2]);

  // THE ASSERTIONS LIVE IN THE EQUATION SECTION. An assertion inside a
  // when-clause is logged and then ignored by the OpenModelica runtime, so a
  // time-windowed claim has to be written as an implication over the whole
  // simulation, exactly as Tests.Cubs2EstimatorTests does.

  // LEFT EQUIVARIANCE. The same sticks flown from a rotated and translated
  // vehicle must produce the rotated and translated reference, exactly. This
  // is the property the group formulation buys: the stick command is a twist
  // in the reference's own frame and the reference advances by right
  // composition, so left multiplication by a fixed transform commutes with
  // the whole update, leash included.
  assert(time < 0.02 or equivarianceResidual < 1.0e-9,
    "Manual guidance is not left equivariant, so the stick response depends on where the vehicle happens to be");

  // LEASH. The vehicle here cannot move, so full stick drives the reference
  // straight into the leash and holds it there.
  assert(time < 0.02 or heldOffset_m < horizontalLeash_m + 1.0e-9,
    "Manual reference ran past the leash");
  assert(time < 1.5 or time >= 3.0
    or heldOffset_m > 0.99 * horizontalLeash_m,
    "Manual reference never reached the leash, so this run does not exercise it");
  assert(time < 1.5 or time >= 3.0 or held.positionWorldEnu_m[1] > 0.0,
    "Full pitch stick did not push the reference along the vehicle heading");
  assert(time < 0.02 or abs(held.positionWorldEnu_m[2]) < 1.0e-9,
    "Pitch stick alone moved the reference sideways");

  // FEED-FORWARD ANTI-WINDUP. The velocity block of the same error logarithm
  // bounds how much faster the reference may be travelling than the vehicle.
  // This vehicle cannot move at all, so the reference speed is exactly that
  // bound; a source without a velocity leash would be feeding the loop the
  // full commanded speed while the vehicle stood still. Expressing this needs
  // velocity inside the group element, which is why the reference is an
  // SE_2(3) extended pose rather than a pose with a velocity beside it.
  assert(time < 0.02 or heldSpeed_m_s < horizontalSpeedLeash_m_s + 1.0e-9,
    "Reference speed ran past the velocity leash, so the feed-forward winds up while the vehicle is held back");
  assert(time < 1.5 or time >= 3.0
    or heldSpeed_m_s > 0.99 * horizontalSpeedLeash_m_s,
    "Reference speed never reached the velocity leash, so this run does not exercise it");

  // RELEASE. With the sticks centered the reference stops; there is no brake
  // phase and no latch, only a commanded velocity that has ramped to zero.
  // Decelerating the velocity leash of 2 m/s at the 3 m/s2 bound, with the
  // jerk ramps at each end, takes about 1.4 s from the release at 3 s. The
  // sticks stay centered for the rest of the run, so the reference has to stay
  // stopped, not merely reach zero once.
  assert(time < 4.6 or heldSpeed_m_s < 1.0e-9,
    "Manual reference did not come to rest and stay there after the sticks were released");

  // BUMPLESS ENTRY AT SPEED. The reference tracks the estimate while the mode
  // is not selected, so on the entry sample it is already at the vehicle and
  // already carries the vehicle's velocity. A source that started from rest
  // would show a step of the full cruise speed here.
  assert(time < 4.0 or time >= 5.0 or enteredPositionError_m < 0.05,
    "Manual reference does not track the estimate while the mode is deselected");
  assert(time < 5.0 or time >= 5.05 or enteredPositionError_m < 0.05,
    "Manual reference jumped away from the vehicle at mode entry");
  assert(time < 5.0 or time >= 5.05 or enteredSpeedError_m_s < 0.05,
    "Manual reference velocity stepped at mode entry instead of inheriting the vehicle velocity");
  // The reference is held between samples while this vehicle keeps moving, so
  // the world-frame gap can exceed the leash by one sample of travel before the
  // next release pulls it back.
  assert(time < 7.0
    or enteredOffset_m
      < horizontalLeash_m + cruiseSpeed_m_s * samplePeriod + 1.0e-9,
    "Manual reference exceeded the leash behind a vehicle that flew away from it");

  // MODE SWITCH. Bands, debounce, hysteresis, and link loss.
  assert(time < 0.02 or time >= 5.0 or selector.mode == 2,
    "Mode switch did not seed the mission mode, or accepted a switch blip shorter than the debounce");
  // This window covers a channel value inside the hysteresis and then a lost
  // link, neither of which may move the mode. A decoder that quantized without
  // hysteresis, or that fell back to a default band on link loss, leaves the
  // pilot position band here.
  assert(time < 5.25 or time >= 7.0 or selector.mode == 3,
    "Mode switch dropped the pilot position band at a channel value inside the hysteresis, or on a lost link");
  assert(time < 7.25 or time >= 8.0 or selector.mode == 1,
    "Mode switch did not leave the band once the channel cleared the hysteresis");
  assert(time < 8.25 or selector.mode == 2,
    "Mode switch did not recover the mission mode after the link returned");
  annotation(experiment(
    StartTime = 0.0, StopTime = 10.0, Tolerance = 1.0e-8, Interval = 0.005));
end ManualGuidanceDynamics;
