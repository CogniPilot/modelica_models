within RigidBody;
record State "Rigid-body state in world position and body velocity coordinates"
  Real worldPosition[3] "World position [m]";
  Real bodyVelocity[3] "Body-frame linear velocity [m/s]";
  Real attitude[4] "Unit quaternion {w,x,y,z}, body to world";
  Real bodyAngularVelocity[3] "Body-frame angular velocity [rad/s]";
end State;
