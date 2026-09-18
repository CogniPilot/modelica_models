within Vehicles.Rdd2;

block NavigationEstimator
  "RDD2 eFMU implementation of the stable navigation estimator boundary"
  extends Estimation.StrapdownINS.ESKF.Estimator(
    samplePeriod = 0.01,
    // Flight-log tuning of the generic strapdown defaults, applied only to
    // the RDD2 deployment through these extends modifiers. The values come
    // from replaying a recorded RDD2 flight through the exported estimator
    // and scoring the aided solution against the GPS fixes.
    processNoise = Estimation.StrapdownINS.ProcessNoise(
      // Gyroscope angular-random-walk density. Raised from the generic
      // 1e-5 because the flight's turn dynamics needed more attitude
      // process noise than the bench default carried for the fixes to keep
      // yaw locked to the ground course.
      gyroscope_rad2_s = identity(3) * 1.0e-4,
      // Accelerometer velocity-random-walk density. The generic 1e-3 left
      // the velocity sigma near 0.05 m/s against a GPS velocity floor of
      // about 0.1 m/s, so each fix was applied with an effective gain near
      // 0.19 and the solution stayed overconfident; 3e-2 opens the gain so
      // the fixes actually move the state, cutting aided horizontal error.
      accelerometer_m2_s3 = identity(3) * 3.0e-2,
      // Gyroscope-bias random-walk density. Lowered from the generic 1e-8
      // to 1e-10 so the gyroscope-bias state cannot chase the per-fix yaw
      // innovations; holding the bias steady kept yaw within about 14
      // degrees of the flown course.
      gyroscopeBias_rad2_s3 = identity(3) * 1.0e-10,
      // Accelerometer-bias random-walk density held at the generic default.
      accelerometerBias_m2_s5 = identity(3) * 1.0e-6),
    initialVariances = Estimation.StrapdownINS.InitialVariances(
      // Position, velocity, attitude, and accelerometer-bias initial
      // variances held at the generic defaults.
      position_m2 = fill(1.0, 3),
      velocity_m2_s2 = fill(1.0, 3),
      attitude_rad2 = fill(0.25, 3),
      // Initial gyroscope-bias variance tightened from the generic 1e-4 to
      // 1e-6 so the filter starts from the well-characterized turn-on bias
      // of the flight IMU rather than admitting a wide early bias swing
      // that the flight data showed disturbing the initial heading.
      gyroscopeBias_rad2_s2 = fill(1.0e-6, 3),
      accelerometerBias_m2_s4 = fill(1.0e-2, 3)),
    // Automatic recovery re-seed enabled for this deployment. On the flight
    // log a 60 s GNSS outage while moving left the solution hundreds of
    // metres off, and every returning fix was gate-rejected, so the ladder
    // stalled at the divergent stage and never recovered. Six seconds is one
    // second beyond the five-second divergent window, so the covariance
    // inflation is given its full span to readmit a good fix on its own
    // before the estimator re-seeds position and velocity from the first
    // fresh fix; on the log this re-anchored within about ten seconds of
    // each outage.
    aidingReseedWindow_s = 6.0,
    initialAlignmentWindow_s = 0.5,
    initialAlignmentTimeout_s = 5.0,
    pseudoPositionVariance_m2 = 100.0,
    zeroVelocityVariance_m2_s2 = 9.0e-2);

  annotation(Documentation(info = "<html>
    <p>Export this block as the default RDD2 estimator eFMU. Alternate filter
    blocks extend <code>Avionics.PartialNavigationEstimator</code>
    and therefore replace this artifact without changing any sensor input,
    navigation output, status field, or controller wiring.</p>
    <p>The process-noise and initial-variance modifiers, the enabled
    automatic re-seed window, the half-second quasi-static alignment window,
    and the synthetic hold-position (10 m sigma) and zero-velocity (0.3 m/s
    sigma) updates that keep an unaided indoor filter bounded are RDD2
    flight-log tuning of the generic strapdown defaults; the defaults in
    <code>Estimation.StrapdownINS.PartialEstimator</code> are unchanged, so
    every other consumer of the filter keeps them.</p>
  </html>"));
end NavigationEstimator;
