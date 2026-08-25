within Tests;
model LogLinearTests "Multirotor log-linear controller tests"
  Control.Multirotor.LogLinear.Controller controller(
    samplePeriod = 0.01,
    mass = 2.0,
    gravity = 9.8,
    thrustTrim = 19.6);

  function run
    output Boolean passed;
  protected
    constant Real tolerance = 2.0e-9;
    constant Real identityQuaternion[4] = {1.0, 0.0, 0.0, 0.0};
    Real error[9];
    Real yawReference[4];
    Real angularVelocity[3];
    Real nextIntegral[3];
    Control.Multirotor.LogLinear.OuterLoopResult hover;
    Control.Multirotor.LogLinear.OuterLoopResult integrated;
    Control.Multirotor.LogLinear.OuterLoopResult translated;
  algorithm
    error := Control.Multirotor.LogLinear.stateError(
      {1.0, 2.0, 3.0},
      {0.5, -0.25, 0.125},
      identityQuaternion,
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      identityQuaternion);
    assert(Tests.Assertions.maxAbsVector(error - {
        -1.0, -2.0, -3.0,
        -0.5, 0.25, -0.125,
        0.0, 0.0, 0.0}) < tolerance,
      "SE_2(3) state error did not use actual^(-1) reference");

    yawReference := {cos(0.1), 0.0, 0.0, sin(0.1)};
    angularVelocity := Control.Multirotor.LogLinear.attitudeControl(
      {2.0, 2.0, 1.0}, identityQuaternion, yawReference);
    assert(Tests.Assertions.maxAbsVector(
        angularVelocity - {0.0, 0.0, 0.2}) < tolerance,
      "SO(3) log-linear attitude feedback was incorrect");

    nextIntegral := Control.Multirotor.LogLinear.integratePositionError(
      {100.0, -100.0, 1.0},
      {0.0, 0.0, 0.0},
      1.0,
      9.8);
    assert(Tests.Assertions.maxAbsVector(
        nextIntegral - {49.0, -49.0, 1.0}) < tolerance,
      "Position-error integral limits were incorrect");

    hover := Control.Multirotor.LogLinear.outerLoop(
      zeros(9),
      zeros(3),
      identityQuaternion,
      identityQuaternion,
      zeros(3),
      19.6);
    assert(abs(hover.thrust - 19.6) < tolerance and
        Tests.Assertions.maxAbsVector(
          hover.attitudeSetpoint - identityQuaternion) < tolerance and
        Tests.Assertions.maxAbsVector(hover.angularVelocityCorrection)
          < tolerance,
      "Log-linear hover command was not the trim command");

    integrated := Control.Multirotor.LogLinear.outerLoop(
      zeros(9),
      zeros(3),
      identityQuaternion,
      identityQuaternion,
      {1.0, 0.0, 0.0},
      19.6);
    assert(integrated.thrust > 19.6 and
        Tests.Assertions.maxAbsVector(
          integrated.attitudeSetpoint - identityQuaternion) > tolerance,
      "Horizontal integral feedback was not applied to the thrust vector");

    translated := Control.Multirotor.LogLinear.outerLoop(
      {0.4, -0.2, 0.1, 0.05, -0.1, 0.0, 0.02, -0.03, 0.01},
      {0.2, -0.1, 0.0},
      identityQuaternion,
      yawReference,
      {0.1, -0.2, 0.3},
      19.6);
    assert(translated.thrust > 0.0 and
        abs(translated.attitudeSetpoint * translated.attitudeSetpoint - 1.0)
          < tolerance and
        Tests.Assertions.isFiniteVector(translated.angularVelocityCorrection),
      "Log-linear controller produced an invalid nonzero command");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  controller.positionWorld = zeros(3);
  controller.velocityWorld = zeros(3);
  controller.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  controller.positionReferenceWorld = {1.0, 0.0, 0.0};
  controller.velocityReferenceWorld = zeros(3);
  controller.accelerationReferenceWorld = zeros(3);
  controller.headingQuaternionReference = {1.0, 0.0, 0.0, 0.0};
  controller.resetIntegral = false;
  assert(passed, "Log-linear controller assertions did not complete");
  assert(controller.thrust > 0.0 and
      abs(controller.attitudeSetpoint * controller.attitudeSetpoint - 1.0)
        < 2.0e-9 and
      Tests.Assertions.isFiniteVector(controller.angularVelocitySetpoint),
    "Sampled log-linear controller produced an invalid command");
end LogLinearTests;
