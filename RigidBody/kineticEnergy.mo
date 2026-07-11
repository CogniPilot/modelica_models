within RigidBody;
function kineticEnergy "Translational plus rotational kinetic storage"
  input RigidBody.State state;
  input RigidBody.Parameters parameters;
  output Real energy;
algorithm
  energy := 0.5 * parameters.mass * (state.bodyVelocity * state.bodyVelocity)
    + 0.5 * state.bodyAngularVelocity
      * (parameters.inertia * state.bodyAngularVelocity);
end kineticEnergy;
