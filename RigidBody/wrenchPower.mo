within RigidBody;
function wrenchPower "Supply rate for a body-frame wrench"
  input RigidBody.State state;
  input RigidBody.Wrench wrench;
  output Real power;
algorithm
  power := RigidBody.portPower(wrench, RigidBody.bodyTwist(state));
end wrenchPower;
