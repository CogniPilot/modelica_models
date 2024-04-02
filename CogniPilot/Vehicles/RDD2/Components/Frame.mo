within CogniPilot.Vehicles.RDD2.Components;

record Frame
  Real r_op_i[3] "position expressed in inertial frame [m]";
  Real v_ip_b[3] "inertial velocity expressed in body frame [m/s]";
  Real a_ip_b[3] "inertial acceleration expressed in body frame [m/s^2]";
  Real w_ib_b[3] "angular velocity of body wrt inertial frame expressed in body frame [rad/s]";
  Real C_ib[3, 3] "rotation matrix from inertial to body frame []";
  Real alpha_ib_b[3] "inertial angular acceleration expressed body frame [rad/s^]";
end Frame;