within RigidBody;
record Wrench "External non-gravity wrench expressed in the body frame"
  Real bodyForce[3] "Force [N]";
  Real bodyTorque[3] "Torque [N*m]";
end Wrench;
