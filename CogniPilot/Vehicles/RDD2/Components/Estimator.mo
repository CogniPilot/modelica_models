within CogniPilot.Vehicles.RDD2.Components;

model Estimator
  ZROS.Bus bus;
  ZROS.SubImu sub_imu;
  ZROS.PubOdometry pub_odometry;
  discrete Real x;
equation
  connect(bus.imu, sub_imu);
  connect(bus.estimator_odometry, pub_odometry);
  pub_odometry.updated = sub_imu.updated;
algorithm
  when edge(sub_imu.updated) then
    x := time;
    pub_odometry.msg.time := time;
    pub_odometry.msg.r_op_i := {time, time, time};
    pub_odometry.msg.v_ip_b := {time, time, time};
    pub_odometry.msg.w_ib_b := {0, 0, 0};
    pub_odometry.msg.C_ib := identity(3);
  end when;
end Estimator;