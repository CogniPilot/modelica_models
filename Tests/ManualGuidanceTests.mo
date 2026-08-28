within Tests;

model ManualGuidanceTests
  "Properties of the pilot stick shaping, ramp, leash, and mode switch"

  constant Real deadband = 0.1;
  constant Real expoFraction = 0.6;
  constant Real switchEdge_us[2] = {1200.0, 1800.0};
  constant Real switchHysteresis_us = 25.0;

  function shapedStick "The stick curve the manual guidance source applies"
    input Real value;
    output Real result;
  algorithm
    result := MathUtilities.expo(
      MathUtilities.deadzone(value, deadband), expoFraction);
  end shapedStick;

  function shapingOddnessResidual
    "Largest failure of odd symmetry over the stick travel"
    output Real residual;
  protected
    constant Integer samples = 81;
    Real position;
  algorithm
    residual := 0.0;
    for index in 1:samples loop
      position := -1.0 + 2.0 * (index - 1) / (samples - 1);
      residual := max(residual,
        abs(shapedStick(position) + shapedStick(-position)));
    end for;
  end shapingOddnessResidual;

  function shapingMinimumSlope
    "Smallest forward difference of the stick curve outside the dead band"
    output Real slope;
  protected
    constant Integer samples = 81;
    constant Real step = 1.0 / (samples - 1);
    Real position;
  algorithm
    slope := 1.0e30;
    for index in 1:samples - 1 loop
      position := deadband + (1.0 - deadband) * (index - 1) * step;
      slope := min(slope,
        (shapedStick(position + step) - shapedStick(position)) / step);
    end for;
  end shapingMinimumSlope;

  function shapingDeadbandEdgeJump
    "Discontinuity of the stick curve at the dead-band edge"
    output Real jump;
  algorithm
    jump := abs(shapedStick(deadband + 1.0e-9) - shapedStick(deadband));
  end shapingDeadbandEdgeJump;

  function rampStop
    "Release one reference axis from the commanded speed and follow it to rest"
    input Boolean useSquareRootRamp
      "False selects the proportional law the ramp replaces";
    input Real samplePeriod(unit = "s");
    input Integer steps;
    output Real excursion "Most negative speed reached";
    output Real residual "Speed left at the end of the run";
  protected
    constant Real releaseSpeed = 5.0;
    constant Real accelerationLimit = 3.0;
    constant Real jerkLimit = 8.0;
    Real speed;
    Real acceleration;
    Real target;
    Real step;
  algorithm
    // One axis of the reference twist, released from the commanded speed with
    // the same acceleration and jerk bounds the manual source uses.
    speed := releaseSpeed;
    acceleration := 0.0;
    excursion := speed;
    for index in 1:steps loop
      target := if useSquareRootRamp then
          MathUtilities.clip(
            MathUtilities.squareRootRamp(
              -speed, 1.0 / samplePeriod, jerkLimit),
            -accelerationLimit, accelerationLimit)
        else
          MathUtilities.clip(
            -speed / samplePeriod, -accelerationLimit, accelerationLimit);
      step := MathUtilities.clip(
        target - acceleration,
        -jerkLimit * samplePeriod, jerkLimit * samplePeriod);
      acceleration := acceleration + step;
      speed := speed + acceleration * samplePeriod;
      excursion := min(excursion, speed);
    end for;
    residual := abs(speed);
  end rampStop;

  function limitNormDirectionResidual
    "Failure of the norm limiter to preserve direction when it binds"
    output Real residual;
  protected
    Real source[2];
    Real limited[2];
  algorithm
    source := {3.0, 4.0};
    limited := MathUtilities.limitNorm(source, 1.0);
    residual := abs(limited[1] * source[2] - limited[2] * source[1])
      + abs(MathUtilities.norm2(limited) - 1.0);
  end limitNormDirectionResidual;

  function switchPosition "Band the switch quantizer reports"
    input Real pwm_us;
    input Real hysteresis_us;
    input Integer previous;
    output Integer position;
  algorithm
    position := Vehicles.Interfaces.switchBandPosition(
      pwm_us, switchEdge_us, hysteresis_us, previous);
  end switchPosition;

  function switchIdempotenceResidual
    "Positions that change when the quantizer is fed its own output"
    output Integer residual;
  protected
    constant Integer samples = 141;
    Real pwm_us;
    Integer once;
    Integer twice;
  algorithm
    residual := 0;
    for index in 1:samples loop
      pwm_us := 800.0 + 10.0 * (index - 1);
      once := switchPosition(pwm_us, switchHysteresis_us, 1);
      twice := switchPosition(pwm_us, switchHysteresis_us, once);
      residual := residual + abs(twice - once);
    end for;
  end switchIdempotenceResidual;

  function switchMonotonicityResidual
    "Band index decreases somewhere as the channel rises"
    output Integer residual;
  protected
    constant Integer samples = 141;
    Real pwm_us;
    Integer previousBand;
    Integer band;
  algorithm
    residual := 0;
    previousBand := switchPosition(800.0, 0.0, 1);
    for index in 2:samples loop
      pwm_us := 800.0 + 10.0 * (index - 1);
      band := switchPosition(pwm_us, 0.0, 1);
      residual := residual + max(0, previousBand - band);
      previousBand := band;
    end for;
  end switchMonotonicityResidual;

  function headingTwistOffAxisMagnitude
    "Roll and pitch content of an extended pose advanced by a heading twist"
    output Real magnitude;
  protected
    Real element[10];
    Real advanced[10];
  algorithm
    element := LieGroups.SE23.Quat.exp_map(
      {1.0, -2.0, 0.5, 0.4, 0.1, -0.2, 0.0, 0.0, 0.9});
    advanced := LieGroups.SE23.Quat.product(
      element,
      LieGroups.SE23.Quat.exp_map(
        {0.3, 0.2, -0.1, 0.05, -0.05, 0.2, 0.0, 0.0, -1.7}));
    // Indices 7 to 10 are {qw, qx, qy, qz}; a pure heading rotation has qx and
    // qy identically zero.
    magnitude := abs(element[8]) + abs(element[9])
      + abs(advanced[8]) + abs(advanced[9]);
  end headingTwistOffAxisMagnitude;

  function leashProjectionResidual
    "Failure of the leashed pose to sit at the leash along the error geodesic"
    input Real leash_m;
    output Real residual;
  protected
    Real vehiclePose[10];
    Real farPose[10];
    Real error[9];
    Real clamped[9];
    Real leashedPose[10];
    Real recovered[9];
  algorithm
    vehiclePose := LieGroups.SE23.Quat.exp_map(
      {2.0, -1.0, 3.0, 0.5, 0.2, -0.1, 0.0, 0.0, 0.7});
    farPose := LieGroups.SE23.Quat.product(
      vehiclePose,
      LieGroups.SE23.Quat.exp_map(
        {30.0, 40.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    error := LieGroups.SE23.Quat.log_map(
      LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(vehiclePose), farPose));
    clamped := cat(1,
      MathUtilities.limitNorm(error[1:2], leash_m),
      {error[3]},
      error[4:6],
      {0.0, 0.0, error[9]});
    leashedPose := LieGroups.SE23.Quat.product(
      vehiclePose, LieGroups.SE23.Quat.exp_map(clamped));
    recovered := LieGroups.SE23.Quat.log_map(
      LieGroups.SE23.Quat.product(
        LieGroups.SE23.Quat.inverse(vehiclePose), leashedPose));
    // The leashed pose must lie at exactly the leash distance and on the same
    // ray of the error logarithm as the unleashed one.
    residual := abs(MathUtilities.norm2(recovered[1:2]) - leash_m)
      + abs(recovered[1] * error[2] - recovered[2] * error[1])
        / MathUtilities.norm2(error[1:2]);
  end leashProjectionResidual;

  // THE ASSERTIONS LIVE IN THE EQUATION SECTION, NOT INSIDE A FUNCTION.
  // OpenModelica folds a no-argument function whose body is constant and
  // discards any assertion inside it with the fold, and it logs and then
  // ignores an assertion inside a when-clause. Only an equation-section
  // assertion ends the run, so the functions above return numbers and the
  // claims are made here. Tests/run-manual-guidance.mos simulates this model
  // as a top-level model for the same reason.
  Real oddnessResidual;
  Real minimumSlope;
  Real deadbandEdgeJump;
  Real squareRootStopExcursion;
  Real proportionalStopExcursion;
  Real stopResidual;
  Real proportionalStopResidual;
  Real fineStopExcursion;
  Real fineStopResidual;
  Real directionResidual;
  Integer idempotenceResidual;
  Integer monotonicityResidual;
  Real offAxisMagnitude;
  Real leashResidual;
equation
  oddnessResidual = shapingOddnessResidual();
  minimumSlope = shapingMinimumSlope();
  deadbandEdgeJump = shapingDeadbandEdgeJump();
  // A coarser step than the flight rate keeps each unrolled loop inside what
  // the compiler folds quickly. The property under test belongs to the ramp,
  // and the pair of rates shows that what reverse travel remains is a
  // discretization residue that shrinks with the step rather than a
  // structural overshoot.
  (squareRootStopExcursion, stopResidual) = rampStop(true, 0.05, 100);
  (fineStopExcursion, fineStopResidual) = rampStop(true, 0.02, 200);
  (proportionalStopExcursion, proportionalStopResidual) =
    rampStop(false, 0.05, 100);
  directionResidual = limitNormDirectionResidual();
  idempotenceResidual = switchIdempotenceResidual();
  monotonicityResidual = switchMonotonicityResidual();
  offAxisMagnitude = headingTwistOffAxisMagnitude();
  leashResidual = leashProjectionResidual(2.0);

  // STICK SHAPING. The endpoints must be exact, or full stick would not reach
  // the configured speed and the parameter would not mean what it says.
  assert(abs(shapedStick(1.0) - 1.0) < 1.0e-12
    and abs(shapedStick(-1.0) + 1.0) < 1.0e-12,
    "Stick shaping does not reach full authority at full stick");
  assert(abs(shapedStick(deadband)) < 1.0e-12
    and abs(shapedStick(0.0)) < 1.0e-12
    and abs(shapedStick(-deadband)) < 1.0e-12,
    "Stick shaping is not zero inside the dead band");
  assert(oddnessResidual < 1.0e-12,
    "Stick shaping is not odd, so the two stick directions differ");
  assert(deadbandEdgeJump < 1.0e-6,
    "Stick shaping steps at the dead-band edge");
  assert(minimumSlope > 0.0,
    "Stick shaping is not monotone, so one commanded speed has two stick positions");

  // THE RAMP. This is the reason the manual source uses a square-root ramp
  // rather than a proportional law: released from the commanded speed under
  // the same acceleration and jerk bounds, the proportional law reverses well
  // past rest because it holds full deceleration until the error is nearly
  // gone and then has to unwind a saturated command through the jerk bound.
  assert(squareRootStopExcursion > -0.1,
    "Reference deceleration reverses past rest, so releasing the sticks flies backwards");
  assert(abs(fineStopExcursion) < 0.6 * abs(squareRootStopExcursion),
    "Reverse travel after release does not shrink with the step, so it is structural rather than discretization");
  assert(proportionalStopExcursion < 4.0 * squareRootStopExcursion,
    "The proportional comparison no longer reverses further than the ramp, so this test no longer discriminates");
  assert(stopResidual < 1.0e-9 and fineStopResidual < 1.0e-9,
    "Reference deceleration does not reach exact rest, so a released stick leaves the reference drifting");

  assert(directionResidual < 1.0e-12,
    "The norm limiter rotates the vector it limits");

  // MODE SWITCH. The hysteresis must make the same channel value decode to
  // different bands depending on the band held, and a memoryless quantizer
  // must disagree, or there is no hysteresis.
  assert(switchPosition(1000.0, switchHysteresis_us, 3) == 1
    and switchPosition(1500.0, switchHysteresis_us, 1) == 2
    and switchPosition(1900.0, switchHysteresis_us, 1) == 3,
    "Mode switch bands do not decode the nominal switch positions");
  assert(switchPosition(1190.0, switchHysteresis_us, 2) == 2
    and switchPosition(1190.0, switchHysteresis_us, 1) == 1
    and switchPosition(1190.0, 0.0, 2) == 1,
    "Mode switch has no hysteresis at the lower band edge");
  assert(switchPosition(1810.0, switchHysteresis_us, 2) == 2
    and switchPosition(1810.0, switchHysteresis_us, 3) == 3
    and switchPosition(1810.0, 0.0, 2) == 3,
    "Mode switch has no hysteresis at the upper band edge");
  assert(switchPosition(1174.0, switchHysteresis_us, 2) == 1
    and switchPosition(1826.0, switchHysteresis_us, 2) == 3,
    "Mode switch does not leave a band once the channel clears the hysteresis");
  assert(switchPosition(1000.0, switchHysteresis_us, 3) == 1,
    "Mode switch cannot cross two bands in one sample");
  assert(idempotenceResidual == 0,
    "Mode switch is not idempotent, so a held channel can keep changing bands");
  assert(monotonicityResidual == 0,
    "Mode switch band index is not monotone in the channel");

  // SCHEDULE LOOKUP used by the scripted transmitter.
  assert(Vehicles.Interfaces.activeScheduleRow(-1.0, {0.0, 1.0, 2.0}) == 1
    and Vehicles.Interfaces.activeScheduleRow(1.5, {0.0, 1.0, 2.0}) == 2
    and Vehicles.Interfaces.activeScheduleRow(9.0, {0.0, 1.0, 2.0}) == 3,
    "Scripted transmitter schedule lookup does not hold the last passed row");

  // GROUP STRUCTURE the manual source relies on. A twist generated only by the
  // world-vertical generator keeps the reference quaternion a pure heading
  // rotation, which is what makes heading wrap-around unrepresentable in the
  // mode rather than something that has to be handled.
  assert(offAxisMagnitude < 1.0e-12,
    "A pure heading twist produced roll or pitch in the reference pose");
  assert(leashResidual < 1.0e-9,
    "The leash is not a projection along the error geodesic");
  annotation(experiment(StartTime = 0.0, StopTime = 0.0));
end ManualGuidanceTests;
