within Tests;
model MultirotorInnerLoopTests "Multirotor body-rate and allocation tests"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-3;
    Real moment[3];
    Real motor[4];
    // RDD2 allocation: inverse of [ones(1,4); RDD2 motor_moment_map].
    constant Real wrenchToThrust[4, 4] = [
      0.25, -1.4142135623730951, -1.4142135623730951, -15.625;
      0.25, -1.4142135623730951,  1.4142135623730951,  15.625;
      0.25,  1.4142135623730951,  1.4142135623730951, -15.625;
      0.25,  1.4142135623730951, -1.4142135623730951,  15.625];
  algorithm
    // A pure roll-rate error drives a single-axis moment via I * k * error.
    moment := Control.Multirotor.RateLoop.bodyMoment(
      {1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      {0.02, 0.02, 0.04},
      {20.0, 20.0, 10.0});
    assert(Tests.Assertions.maxAbsVector(moment - {0.4, 0.0, 0.0}) < tolerance,
      "Body-rate moment did not follow I * gain * rate error");

    // Hover on the RDD2 airframe: 19.6 N split evenly is 4.9 N per rotor, which
    // is 0.688 normalized against the 1100 rad/s rotor limit.
    motor := Control.Multirotor.Allocation.motorCommands(
      19.6, {0.0, 0.0, 0.0}, wrenchToThrust, 8.54858e-6, 1100.0);
    assert(Tests.Assertions.maxAbsVector(motor - {
        0.6882, 0.6882, 0.6882, 0.6882}) < 2.0e-3,
      "Hover allocation did not match the RDD2 hover throttle");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Multirotor inner-loop assertions did not complete");
  annotation(experiment(StartTime = 0.0, StopTime = 1.0, Tolerance = 1.0e-8,
    Interval = 0.001));
end MultirotorInnerLoopTests;
