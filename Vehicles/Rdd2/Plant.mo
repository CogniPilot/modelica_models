within Vehicles.Rdd2;
model Plant "RDD2 quadrotor parameterization"
  extends Vehicles.Templates.QuadrotorPlant(
    vehicle_mass = 2.0,
    gravity = 9.80665,
    vehicle_ixx = 0.02166666666666667,
    vehicle_iyy = 0.02166666666666667,
    vehicle_izz = 0.04000000000000001,
    arm_length = 0.25,
    Ct = 8.54858e-6,
    Cm = 0.016,
    motor_moment_map = [
      -d, -d, d, d;
      -d, d, d, -d;
      -Cm, Cm, -Cm, Cm
    ]
  );
end Plant;
