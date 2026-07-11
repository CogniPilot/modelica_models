within Estimation.MocapExternalOdometryIEKF;
record PoseMeasurement "Position and attitude observation"
  Real position[3] "Measured position in world ENU coordinates [m]";
  Real attitude[4] "Measured scalar-first Hamilton quaternion {w,x,y,z}";
end PoseMeasurement;
