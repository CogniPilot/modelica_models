within SLAM.Examples;
model ResponsiveInertial "Short IMU filters: faster response, less noise rejection"
  import InertialOnly = SLAM.Examples.InertialOnly;

  extends InertialOnly(
    accelTimeConstant=0.005,
    gyroTimeConstant=0.005);

  annotation (experiment(StopTime=10, Interval=0.01, Tolerance=1e-8));
end ResponsiveInertial;
