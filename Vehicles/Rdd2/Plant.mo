within Vehicles.Rdd2;

model Plant "RDD2 quadrotor with native avionics-facing connectors"
  parameter Real omegaMax(unit = "rad/s") = 1100.0;
  parameter Vehicles.Rdd2.RotorGeometry rotorGeometry =
    Vehicles.Rdd2.RotorGeometry();
  parameter Real rotorEffectiveness[4, 4] =
    Control.Multirotor.Allocation.rotorEffectiveness(
      rotorGeometry.positionBodyFlu_m,
      rotorGeometry.yawMomentPerThrust_m);

  Vehicles.Rdd2.ControllerInterfaces.MotorCommandInput commands;
  Avionics.ImuSampleOutput imu;
  Avionics.NavigationEstimateOutput truth;
  output Real motorOmega_rad_s[4];

protected
  Vehicles.Templates.QuadrotorPlant dynamics(
    vehicle_mass = 2.0,
    gravity = 9.80665,
    vehicle_ixx = 0.02166666666666667,
    vehicle_iyy = 0.02166666666666667,
    vehicle_izz = 0.04000000000000001,
    arm_length = rotorGeometry.armLength_m,
    Ct = 8.54858e-6,
    Cm = rotorGeometry.rotorTorqueRatio_m,
    motor_moment_map = rotorEffectiveness[2:4, :]
  );
  Real eulerB321_rad[3] "Lie-group B321 coordinates [yaw, pitch, roll]";

equation
  dynamics.omega_cmd = omegaMax * {
    MathUtilities.clip(commands.motor[motorIndex], 0.0, 1.0)
      for motorIndex in 1:4};

  imu.valid = true;
  imu.fresh = true;
  imu.timestamp_s = time;
  imu.angularVelocityBodyFlu_rad_s = dynamics.gyro;
  imu.specificForceBodyFlu_m_s2 = dynamics.accel;

  eulerB321_rad = LieGroups.SO3.EulerB321.from_Quat(dynamics.quat);
  truth.valid = true;
  truth.timestamp_s = time;
  truth.positionWorldEnu_m = dynamics.position;
  truth.velocityWorldEnu_m_s = dynamics.velocity;
  truth.accelerationWorldEnu_m_s2 =
    dynamics.R * dynamics.accel + {0.0, 0.0, -dynamics.gravity};
  truth.quaternionWorldBody = dynamics.quat;
  truth.rotationWorldBody = dynamics.R;
  truth.eulerRpy_rad = {
    eulerB321_rad[3], eulerB321_rad[2], eulerB321_rad[1]};
  truth.angularVelocityBodyFlu_rad_s = dynamics.gyro;
  truth.angularVelocityWorldEnu_rad_s = dynamics.R * dynamics.gyro;
  motorOmega_rad_s = dynamics.omega_m;
end Plant;
