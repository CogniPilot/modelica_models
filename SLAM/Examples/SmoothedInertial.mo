within SLAM.Examples;
model SmoothedInertial "Long IMU filters: more noise rejection, more delay"
  import InertialOnly = SLAM.Examples.InertialOnly;

  extends InertialOnly(
    accelTimeConstant=0.12,
    gyroTimeConstant=0.08);

  annotation (experiment(StopTime=10, Interval=0.01, Tolerance=1e-8));
end SmoothedInertial;
