within Tests;

model Cubs2EstimatorTests
  "Mocap dropout propagation and bounded recovery without estimator modes"
  Vehicles.Cubs2.OuterLoopComponents.StateEstimator estimator(
    dt = 0.02,
    filterCutoffHz = 25.0,
    velocityFilterCutoffHz = 25.0,
    accelFilterCutoffHz = 5.0);

  Real truePosition_m[3];
  Real trueEuler_rad[3];
  Real mocapPosition_m[3];
  Real mocapEuler_rad[3];

equation
  truePosition_m = {4.0 * time, 0.0, 3.0};
  trueEuler_rad = {0.0, 0.0, 0.2 * time};
  if time < 0.3 or time >= 0.6 then
    mocapPosition_m = truePosition_m;
    mocapEuler_rad = trueEuler_rad;
  else
    mocapPosition_m = {1.2, 0.0, 3.0};
    mocapEuler_rad = {0.0, 0.0, 0.06};
  end if;

  estimator.position_m = mocapPosition_m;
  estimator.euler_rad = mocapEuler_rad;
  estimator.velocity_m_s = {4.0, 0.0, 0.0};
  estimator.eulerRate_rad_s = {0.0, 0.0, 0.2};

algorithm
  when time >= 0.5 then
    assert(estimator.measurementWeight < 1.0e-6,
      "A repeated mocap packet received nonzero correction weight");
    assert(estimator.estimate.position_m[1] > 1.8,
      "The estimator stopped propagating position during mocap dropout");
    assert(estimator.estimate.euler_rad[3] > 0.08,
      "The estimator stopped propagating attitude during mocap dropout");
  end when;
  when time >= 0.8 then
    assert(abs(estimator.estimate.position_m[1] - truePosition_m[1]) < 0.25,
      "The estimator did not recover smoothly after mocap returned");
    assert(abs(estimator.estimate.euler_rad[3] - trueEuler_rad[3]) < 0.03,
      "The attitude estimate did not recover after mocap returned");
  end when;
end Cubs2EstimatorTests;
