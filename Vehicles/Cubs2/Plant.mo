within Vehicles.Cubs2;
model Plant "CUBS2 Sport Cub parameterization"
  extends Vehicles.Templates.FixedWingPlant(
    vehicle_mass = 0.065,
    gravity = 9.81,
    Jx = 8.0e-4,
    Jy = 1.2e-3,
    Jz = 1.8e-3,
    Jxz = 1.0e-4,
    S = 0.055,
    span = 0.617,
    cbar = 0.09,
    thr_max = 0.30,
    wheel_x = {0.1, 0.1, -0.4},
    wheel_y = {0.1, -0.1, 0.0},
    wheel_z = {-0.1, -0.1, 0.0}
  );
end Plant;
