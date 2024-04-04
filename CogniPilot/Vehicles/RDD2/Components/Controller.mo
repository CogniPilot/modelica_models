within CogniPilot.Vehicles.RDD2.Components;
model Controller
  ZROS.Bus bus;

  ZROS.SubOdometry sub_odometry;

  ZROS.PubActuators pub_actuators;

  parameter Real omega0=200;

  Real x;

  constant Real pi=3.14159;

equation
  connect(bus.estimator_odometry,sub_odometry);

  connect(bus.actuators,pub_actuators);

  pub_actuators.updated=sub_odometry.updated;

algorithm
  x := sub_odometry.msg.r_op_i[1];

  when edge(
    sub_odometry.updated) then
    pub_actuators.msg.time := time;

    pub_actuators.msg.vel := {1+omega0,1+omega0,-1+omega0,-1+omega0};

  end when;

end Controller;
