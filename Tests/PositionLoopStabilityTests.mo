within Tests;

model PositionLoopStabilityTests
  "Closed-loop stability of the log-linear horizontal position loop"

  function commandedPitchRate
    "Body pitch rate the shipped control law commands for one hover state"
    input Real positionErrorEast_m;
    input Real velocityErrorEast_m_s;
    input Real pitch_rad;
    input Real mass;
    input Real gravity;
    input Real thrustTrim;
    input Real attitudeGain[3];
    output Real pitchRate_rad_s;
  protected
    Real quaternionWorldBody[4];
    Real error[9];
    Control.Multirotor.LogLinear.OuterLoopResult result;
    Real angularVelocity[3];
  algorithm
    // Pitch about body y tilts world z toward world x, which is the axis the
    // east position loop closes through.
    quaternionWorldBody := {cos(0.5 * pitch_rad), 0.0, sin(0.5 * pitch_rad),
      0.0};
    error := Control.Multirotor.LogLinear.stateError(
      {positionErrorEast_m, 0.0, 0.0},
      {velocityErrorEast_m_s, 0.0, 0.0},
      quaternionWorldBody,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0, 0.0});
    result := Control.Multirotor.LogLinear.outerLoop(
      error,
      {0.0, 0.0, 0.0},
      quaternionWorldBody,
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      thrustTrim,
      mass,
      gravity);
    angularVelocity := Control.Multirotor.LogLinear.attitudeControl(
      attitudeGain, quaternionWorldBody, result.attitudeSetpoint)
      + result.angularVelocityCorrection;
    pitchRate_rad_s := angularVelocity[2];
  end commandedPitchRate;

  function routhCoefficients
    "Coefficients of s^3 + a2 s^2 + a1 s + a0 for the closed east loop"
    output Real coefficient[3] "a2, a1, a0";
  protected
    // RDD2 parameterization, matching Vehicles.Rdd2.LogLinearController.
    constant Real mass = 2.0;
    constant Real gravity = 9.8;
    constant Real thrustTrim = 19.6;
    constant Real attitudeGain[3] = {2.0, 2.0, 1.0};
    constant Real step = 1.0e-4;
    Real positionSensitivity;
    Real velocitySensitivity;
    Real pitchSensitivity;
  algorithm
    // Numerically linearize the shipped control law about hover. The plant
    // rows are exact: d(position)/dt = velocity, and d(velocity)/dt =
    // gravity * pitch for a thrust-vectored vehicle holding hover thrust.
    positionSensitivity := (
      commandedPitchRate(step, 0.0, 0.0, mass, gravity, thrustTrim,
        attitudeGain)
      - commandedPitchRate(-step, 0.0, 0.0, mass, gravity, thrustTrim,
        attitudeGain)) / (2.0 * step);
    velocitySensitivity := (
      commandedPitchRate(0.0, step, 0.0, mass, gravity, thrustTrim,
        attitudeGain)
      - commandedPitchRate(0.0, -step, 0.0, mass, gravity, thrustTrim,
        attitudeGain)) / (2.0 * step);
    pitchSensitivity := (
      commandedPitchRate(0.0, 0.0, step, mass, gravity, thrustTrim,
        attitudeGain)
      - commandedPitchRate(0.0, 0.0, -step, mass, gravity, thrustTrim,
        attitudeGain)) / (2.0 * step);

    // Characteristic polynomial of the closed loop
    //   [0 1 0; 0 0 g; positionSensitivity velocitySensitivity pitchSensitivity]
    coefficient[1] := -pitchSensitivity;
    coefficient[2] := -gravity * velocitySensitivity;
    coefficient[3] := -gravity * positionSensitivity;
  end routhCoefficients;

  function massInvarianceResidual
    "Difference in commanded pitch rate between a 1 kg and a 4 kg vehicle"
    output Real residual_rad_s;
  protected
    constant Real gravity = 9.8;
    constant Real attitudeGain[3] = {2.0, 2.0, 1.0};
  algorithm
    residual_rad_s :=
      commandedPitchRate(0.1, 0.0, 0.0, 1.0, gravity, 1.0 * gravity,
        attitudeGain)
      - commandedPitchRate(0.1, 0.0, 0.0, 4.0, gravity, 4.0 * gravity,
        attitudeGain);
  end massInvarianceResidual;

  // THE ASSERTIONS LIVE IN THE EQUATION SECTION, NOT INSIDE A FUNCTION.
  //
  // An assertion inside a no-argument function whose body OpenModelica can
  // fold to a constant is discarded with the fold: the call never reaches
  // generated code, and a violated condition then costs nothing. Measured on
  // this very model -- with all three Routh conditions rewritten to the
  // impossible `> 1.0e9`, the model still simulated successfully and the
  // suite still reported green. The same is true of an assert in any
  // when-clause, which the runtime logs and then ignores. Only an assert in
  // the equation section actually ends the run, so that is where a
  // qualification claim belongs, with the function reduced to returning the
  // numbers it measured.
  Real coefficient[3];
  Real a2;
  Real a1;
  Real a0;
  Real massResidual_rad_s;
equation
  coefficient = routhCoefficients();
  a2 = coefficient[1];
  a1 = coefficient[2];
  a0 = coefficient[3];
  massResidual_rad_s = massInvarianceResidual();

  // ROUTH-HURWITZ FOR A CUBIC. All three conditions are required, and a0 > 0
  // is the one that failed when the translational feedback was added to the
  // thrust vector without the mass factor while the rotation rows of the LQR
  // gain were added to the body-rate command unscaled. That combination left
  // a0 = -0.98 at this vehicle's 2 kg mass: one real pole at +0.042 rad/s,
  // and a station-keeping divergence with a time constant near 24 s. The
  // qualification missions run 45 s against a reference that is always
  // moving, so they never showed it.
  //
  // The sensitivities are measured from the shipped functions rather than
  // restated here, so any change to the gains, to the mass scaling, or to the
  // rate command that reintroduces a right-half-plane pole fails here.
  assert(a2 > 0.0,
    "Horizontal position loop: pitch damping is not positive");
  assert(a0 > 0.0,
    "Horizontal position loop: position feedback has the wrong sign, so the loop is divergent at this vehicle mass");
  assert(a2 * a1 > a0,
    "Horizontal position loop: Routh-Hurwitz product condition violated");

  // The loop must also be mass-independent. A tilt command is an acceleration
  // command, so the same error must produce the same rate at any mass; if it
  // does not, the law is only stable at its tuned mass.
  assert(abs(massResidual_rad_s) < 1.0e-9,
    "Horizontal position loop response changed with vehicle mass");
  annotation(experiment(StartTime = 0.0, StopTime = 0.0));
end PositionLoopStabilityTests;
