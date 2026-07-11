within RigidBody;
function potentialEnergy "Uniform-gravity potential energy with world z upward"
  input RigidBody.State state;
  input RigidBody.Parameters parameters;
  output Real energy;
algorithm
  energy := parameters.mass * parameters.gravity * state.worldPosition[3];
end potentialEnergy;
