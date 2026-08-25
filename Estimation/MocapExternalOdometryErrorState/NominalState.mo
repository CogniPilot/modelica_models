within Estimation.MocapExternalOdometryErrorState;
record NominalState "Nominal rigid-body state carried by the geometric error-state EKF"
  Real attitude[4] "Scalar-first Hamilton quaternion {w,x,y,z}";
  Real velocity[3] "Linear velocity in world ENU coordinates [m/s]";
  Real position[3] "Position in world ENU coordinates [m]";
  Real angularVelocity[3] "Angular velocity in body FLU coordinates [rad/s]";
end NominalState;
