within CogniPilot.ZROS;
record MsgOdometry
  Real t;

  Real r_op_i[3]
    "position expressed in inertial frame [m]";

  Real v_ip_b[3]
    "inertial velocity expressed in body frame [m/s]";

  Real w_ib_b[3]
    "angular velocity of body wrt inertial frame expressed in body frame [rad/s]";

  Real C_ib[3,3]
    "rotation matrix from inertial to body frame []";

end MsgOdometry;
