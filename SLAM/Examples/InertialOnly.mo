within SLAM.Examples;
model InertialOnly "Inertial navigation with filtered airframe IMU measurements"
  import ModelicaInertial = SLAM.Inertial.ModelicaInertial;

  parameter Real accelTimeConstant(unit="s", min=1e-6) = 0.03
    "Accelerometer filter time constant";
  parameter Real gyroTimeConstant(unit="s", min=1e-6) = 0.02
    "Gyroscope filter time constant";

  input Real accel[3](each unit="m/s2", start={0, 0, 9.81})
    "Specific force in body FLU coordinates";
  input Real gyro[3](each unit="rad/s", each start=0)
    "Angular velocity in body FLU coordinates";

  output Real position[3](each unit="m") "Position in world ENU coordinates";
  output Real velocity[3](each unit="m/s") "Velocity in world ENU coordinates";
  output Real quaternion[4] "Unit quaternion {w,x,y,z}, body to world";
  output Real filteredAccel[3](each unit="m/s2") "Filtered specific force";
  output Real filteredGyro[3](each unit="rad/s") "Filtered angular velocity";

  ModelicaInertial estimator(
    accel_tau=accelTimeConstant,
    gyro_tau=gyroTimeConstant,
    accel=accel,
    gyro=gyro);
equation
  position = estimator.position;
  velocity = estimator.velocity;
  quaternion = estimator.quaternion;
  filteredAccel = estimator.filteredAccel;
  filteredGyro = estimator.filteredGyro;

  annotation (experiment(StopTime=10, Interval=0.01, Tolerance=1e-8));
end InertialOnly;
