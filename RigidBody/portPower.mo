within RigidBody;
function portPower "Bilinear wrench/twist supply rate"
  input RigidBody.Wrench wrench;
  input RigidBody.Twist twist;
  output Real power;
algorithm
  power := wrench.bodyForce * twist.bodyLinearVelocity
    + wrench.bodyTorque * twist.bodyAngularVelocity;
end portPower;
