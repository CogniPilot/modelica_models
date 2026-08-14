within Tests;
model MultirotorInnerLoopTests "Multirotor body-rate and allocation tests"
  function run
    output Boolean passed;
  protected
    constant Real tolerance = 1.0e-3;
    Real moment[3];
    Real motor[4];
    Real hexRotor[6];
    Real wrenchToThrust[4, 4];
    Real effectiveness[4, 4];
    Vehicles.Rdd2.RotorGeometry rotorGeometry;
  algorithm
    // A pure roll-rate error drives a single-axis moment via I * k * error.
    moment := Control.Multirotor.RateLoop.bodyMoment(
      {1.0, 0.0, 0.0},
      {0.0, 0.0, 0.0},
      {0.02, 0.02, 0.04},
      {20.0, 20.0, 10.0});
    assert(Tests.Assertions.maxAbsVector(moment - {0.4, 0.0, 0.0}) < tolerance,
      "Body-rate moment did not follow I * gain * rate error");

    effectiveness := Control.Multirotor.Allocation.rotorEffectiveness(
      rotorGeometry.positionBodyFlu_m,
      rotorGeometry.yawMomentPerThrust_m);
    wrenchToThrust := Control.Multirotor.Allocation.quadrotorWrenchToThrust(
      rotorGeometry.positionBodyFlu_m,
      rotorGeometry.yawMomentPerThrust_m);
    assert(Tests.Assertions.maxAbsMatrix(
        effectiveness * wrenchToThrust - identity(4)) < 1.0e-10,
      "Geometry-derived RDD2 allocation is not the effectiveness inverse");

    // Hover on the RDD2 airframe: 19.6 N split evenly is 4.9 N per rotor, which
    // is 0.688 normalized against the 1100 rad/s rotor limit.
    motor := Control.Multirotor.Allocation.rotorCommands(
      4,
      19.6,
      {0.0, 0.0, 0.0},
      wrenchToThrust,
      fill(8.54858e-6, 4),
      fill(1100.0, 4));
    assert(Tests.Assertions.maxAbsVector(motor - {
        0.6882, 0.6882, 0.6882, 0.6882}) < 2.0e-3,
      "Hover allocation did not match the RDD2 hover throttle");

    hexRotor := Control.Multirotor.Allocation.rotorCommands(
      6,
      12.0,
      zeros(3),
      [1.0 / 6.0, 0.0, 0.0, 0.0;
       1.0 / 6.0, 0.0, 0.0, 0.0;
       1.0 / 6.0, 0.0, 0.0, 0.0;
       1.0 / 6.0, 0.0, 0.0, 0.0;
       1.0 / 6.0, 0.0, 0.0, 0.0;
       1.0 / 6.0, 0.0, 0.0, 0.0],
      fill(1.0, 6),
      fill(2.0, 6));
    assert(Tests.Assertions.maxAbsVector(
        hexRotor - fill(1.0 / sqrt(2.0), 6)) < tolerance,
      "Allocator did not preserve a six-rotor tensor");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Multirotor inner-loop assertions did not complete");
  annotation(experiment(StartTime = 0.0, StopTime = 1.0, Tolerance = 1.0e-8,
    Interval = 0.001));
end MultirotorInnerLoopTests;
