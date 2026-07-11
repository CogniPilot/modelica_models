within RigidBody;
function bodyTwist "Extract the power-port flow variable from a rigid-body state"
  input RigidBody.State state;
  output RigidBody.Twist twist;
algorithm
  twist.bodyLinearVelocity := state.bodyVelocity;
  twist.bodyAngularVelocity := state.bodyAngularVelocity;
end bodyTwist;
