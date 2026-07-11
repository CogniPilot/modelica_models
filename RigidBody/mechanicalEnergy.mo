within RigidBody;
function mechanicalEnergy "Rigid-body mechanical storage function"
  input RigidBody.State state;
  input RigidBody.Parameters parameters;
  output Real energy;
algorithm
  energy := RigidBody.kineticEnergy(state, parameters)
    + RigidBody.potentialEnergy(state, parameters);
end mechanicalEnergy;
