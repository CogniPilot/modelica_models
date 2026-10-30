within Vehicles.Rdd2.Test;
model Rdd2RateFeedforwardChecks
  "Independent axis/frame and zero-thrust consumer controls"
  Vehicles.Rdd2.GuidanceController eastJerk;
  Vehicles.Rdd2.GuidanceController yawed;
  Vehicles.Rdd2.GuidanceController manual;
  Vehicles.Rdd2.GuidanceController freefall;
  Vehicles.Rdd2.GuidanceController lowThrust;
  Vehicles.Rdd2.GuidanceController tilted;
  Vehicles.Rdd2.GuidanceController disabled(maximumTrajectoryFeedforwardRate_rad_s=0.0);
equation
  eastJerk.mode = 2;
  eastJerk.armed = true;
  eastJerk.pilot.stick = zeros(3);
  eastJerk.pilot.throttle = 0.0;
  eastJerk.navigation.positionWorldEnu_m = zeros(3);
  eastJerk.navigation.velocityWorldEnu_m_s = zeros(3);
  eastJerk.navigation.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  eastJerk.reference.positionWorld_m = zeros(3);
  eastJerk.reference.velocityWorld_m_s = zeros(3);
  eastJerk.reference.accelerationWorld_m_s2 = zeros(3);
  eastJerk.reference.jerkWorld_m_s3 = {1.0, 0.0, 0.0};
  eastJerk.reference.yaw_rad = 0.0;
  eastJerk.reference.yawRate_rad_s = 0.0;
  // At level hover d(pitch)/dt = d(acceleration_east)/dt / g.
  assert(abs(eastJerk.angularVelocitySetpointFlu_rad_s[1]) < 1.0e-8 and
    abs(eastJerk.angularVelocitySetpointFlu_rad_s[2] - 1.0 / 9.8) < 1.0e-8 and
    abs(eastJerk.angularVelocitySetpointFlu_rad_s[3]) < 1.0e-8,
    "East jerk must command positive body pitch rate at hover");

  yawed.mode = 2;
  yawed.armed = true;
  yawed.pilot.stick = zeros(3);
  yawed.pilot.throttle = 0.0;
  yawed.navigation.positionWorldEnu_m = zeros(3);
  yawed.navigation.velocityWorldEnu_m_s = zeros(3);
  yawed.navigation.quaternionWorldBody = {sqrt(0.5), 0.0, 0.0, sqrt(0.5)};
  yawed.reference.positionWorld_m = zeros(3);
  yawed.reference.velocityWorld_m_s = zeros(3);
  yawed.reference.accelerationWorld_m_s2 = zeros(3);
  yawed.reference.jerkWorld_m_s3 = {1.0, 0.0, 0.0};
  yawed.reference.yaw_rad = 2.0 * asin(1.0) / 2.0;
  yawed.reference.yawRate_rad_s = 0.3;
  assert(abs(yawed.angularVelocitySetpointFlu_rad_s[1] - 1.0 / 9.8) < 1.0e-8 and
    abs(yawed.angularVelocitySetpointFlu_rad_s[2]) < 1.0e-8 and
    abs(yawed.angularVelocitySetpointFlu_rad_s[3] - 0.3) < 1.0e-8,
    "Yawed vehicle must receive rate in measured body axes, including yaw feed-forward");

  manual.mode = 0;
  manual.armed = true;
  manual.pilot.stick = zeros(3);
  manual.pilot.throttle = 0.0;
  manual.navigation.positionWorldEnu_m = zeros(3);
  manual.navigation.velocityWorldEnu_m_s = zeros(3);
  manual.navigation.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  manual.reference.positionWorld_m = zeros(3);
  manual.reference.velocityWorld_m_s = zeros(3);
  manual.reference.accelerationWorld_m_s2 = {0.0, 0.0, -9.8};
  manual.reference.jerkWorld_m_s3 = {1.0, 2.0, 3.0};
  manual.reference.yaw_rad = 0.0;
  manual.reference.yawRate_rad_s = 0.3;
  assert(sum(abs(manual.angularVelocitySetpointFlu_rad_s)) < 1.0e-8,
    "Acro must not consume mission rate feed-forward or assert at zero thrust");

  freefall.mode = 2;
  freefall.armed = true;
  freefall.pilot.stick = zeros(3);
  freefall.pilot.throttle = 0.0;
  freefall.navigation.positionWorldEnu_m = zeros(3);
  freefall.navigation.velocityWorldEnu_m_s = zeros(3);
  freefall.navigation.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  freefall.reference.positionWorld_m = zeros(3);
  freefall.reference.velocityWorld_m_s = zeros(3);
  freefall.reference.accelerationWorld_m_s2 = {0.0, 0.0, -9.8};
  freefall.reference.jerkWorld_m_s3 = {1.0, 0.0, 0.0};
  freefall.reference.yaw_rad = 0.0;
  freefall.reference.yawRate_rad_s = 0.0;
  assert(sum(abs(freefall.angularVelocitySetpointFlu_rad_s)) < 1.0e-8,
    "Zero-thrust mission must preserve the existing outer-loop fallback");
  lowThrust.mode = 2;
  lowThrust.armed = true;
  lowThrust.pilot.stick = zeros(3);
  lowThrust.pilot.throttle = 0.0;
  lowThrust.navigation.positionWorldEnu_m = zeros(3);
  lowThrust.navigation.velocityWorldEnu_m_s = zeros(3);
  lowThrust.navigation.quaternionWorldBody = {1.0,0.0,0.0,0.0};
  lowThrust.reference.positionWorld_m = zeros(3);
  lowThrust.reference.velocityWorld_m_s = zeros(3);
  lowThrust.reference.accelerationWorld_m_s2 = {0.0,0.0,-9.798};
  lowThrust.reference.jerkWorld_m_s3 = {1.0,0.0,0.0};
  lowThrust.reference.yaw_rad = 0.0;
  lowThrust.reference.yawRate_rad_s = 0.0;
  tilted.mode = 2;
  tilted.armed = true;
  tilted.pilot.stick = zeros(3);
  tilted.pilot.throttle = 0.0;
  tilted.navigation.positionWorldEnu_m = zeros(3);
  tilted.navigation.velocityWorldEnu_m_s = zeros(3);
  tilted.navigation.quaternionWorldBody = {cos(0.15),sin(0.15),0.0,0.0};
  tilted.reference.positionWorld_m = zeros(3);
  tilted.reference.velocityWorld_m_s = zeros(3);
  tilted.reference.accelerationWorld_m_s2 = zeros(3);
  tilted.reference.jerkWorld_m_s3 = {1.0,0.0,0.0};
  tilted.reference.yaw_rad = 0.0;
  tilted.reference.yawRate_rad_s = 0.3;
  disabled.mode = 2;
  disabled.armed = true;
  disabled.pilot.stick = zeros(3);
  disabled.pilot.throttle = 0.0;
  disabled.navigation.positionWorldEnu_m = zeros(3);
  disabled.navigation.velocityWorldEnu_m_s = zeros(3);
  disabled.navigation.quaternionWorldBody = {1.0,0.0,0.0,0.0};
  disabled.reference.positionWorld_m = zeros(3);
  disabled.reference.velocityWorld_m_s = zeros(3);
  disabled.reference.accelerationWorld_m_s2 = zeros(3);
  disabled.reference.jerkWorld_m_s3 = {1.0,0.0,0.0};
  disabled.reference.yaw_rad = 0.0;
  disabled.reference.yawRate_rad_s = 0.0;
  // Specific thrust 0.002 m/s2 is just above the reconstruction guard;
  // unbounded east-jerk feed-forward would be 500 rad/s.
  assert(abs(lowThrust.angularVelocitySetpointFlu_rad_s[1]) < 1.0e-8 and
    abs(lowThrust.angularVelocitySetpointFlu_rad_s[2] - 3.5) < 1.0e-8 and
    abs(lowThrust.angularVelocitySetpointFlu_rad_s[3]) < 1.0e-8,
    "Near-zero-thrust trajectory rate must be bounded with direction preserved");
  // Measured body is rolled 0.3 rad, desired flatness frame is level.
  // Feedback is -6*0.3 on x; R_x(-0.3) transports {0,1/g,0.3}.
  assert(abs(tilted.angularVelocitySetpointFlu_rad_s[1] + 1.8) < 1.0e-8 and
    abs(tilted.angularVelocitySetpointFlu_rad_s[2]
      - cos(0.3)/9.8 - 0.3*sin(0.3)) < 1.0e-8 and
    abs(tilted.angularVelocitySetpointFlu_rad_s[3]
      + sin(0.3)/9.8 - 0.3*cos(0.3)) < 1.0e-8,
    "Feed-forward must be transported into the nonidentity measured frame");
  assert(sum(abs(disabled.angularVelocitySetpointFlu_rad_s)) < 1.0e-8,
    "Zero trajectory feed-forward limit must disable only that term");
  annotation(experiment(StartTime=0.0, StopTime=0.02, Interval=0.005));
end Rdd2RateFeedforwardChecks;
