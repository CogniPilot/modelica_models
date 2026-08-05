within Estimation.PositionVelocityKF;
record State "World-frame translational estimator state"
  Real position[3] "Position in world ENU coordinates [m]";
  Real velocity[3] "Velocity in world ENU coordinates [m/s]";
end State;
