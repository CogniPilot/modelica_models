within RigidBody;
record Twist "Body-frame flow variable power-conjugate to RigidBody.Wrench"
  Real bodyLinearVelocity[3] "Linear velocity [m/s]";
  Real bodyAngularVelocity[3] "Angular velocity [rad/s]";
end Twist;
