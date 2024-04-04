within CogniPilot.Vehicles.RDD2.Components;
model IMU
  parameter Real updateRate=200;
  ZROS.Bus bus;
  ZROS.PubImu pub_imu;
  InFrame frame;
equation
  connect(bus.imu,pub_imu);
  pub_imu.updated=sample(
    0,
    1/updateRate);
algorithm
  when edge(
    pub_imu.updated) then
    pub_imu.msg.time := time;
    pub_imu.msg.a_ip_b := frame.a_ip_b;
    pub_imu.msg.w_ib_b := frame.w_ib_b;
  end when;
end IMU;
