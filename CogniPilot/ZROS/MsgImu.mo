within CogniPilot.ZROS;
record MsgImu
  Real time;
  Real a_ip_b[3]
    "inertial acceleration expressed body frame [m/s^2]";
  Real w_ib_b[3]
    "inertial acceleration expressed body frame [m/s^2]";
end MsgImu;
