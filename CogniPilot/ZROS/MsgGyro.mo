within CogniPilot.ZROS;
record MsgGyro
  Real t;

  Real omega_ib_b[3]
    "inertial angular acceleration expressed body frame [rad/s]";

end MsgGyro;
