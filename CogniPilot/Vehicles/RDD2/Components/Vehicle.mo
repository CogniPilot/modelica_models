within CogniPilot.Vehicles.RDD2.Components;
model Vehicle
  ZROS.Bus bus;

  IMU imu;

  Estimator estimator;

  Controller controller;

  Dynamics dynamics;

equation
  connect(bus,imu.bus);

  connect(bus,estimator.bus);

  connect(bus,controller.bus);

  connect(bus,dynamics.bus);

  connect(imu.frame,dynamics.frame);

end Vehicle;
