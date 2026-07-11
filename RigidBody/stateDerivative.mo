within RigidBody;
function stateDerivative
  "Pure rigid-body vector field shared by simulation and formal analysis"
  input RigidBody.State state;
  input RigidBody.Wrench wrench;
  input RigidBody.Parameters parameters;
  output RigidBody.StateDerivative derivative;
algorithm
  (derivative.worldPosition,
   derivative.bodyVelocity,
   derivative.attitude,
   derivative.bodyAngularVelocity) := RigidBody.stateDerivativeComponents(
    RigidBody.State(
      worldPosition=state.worldPosition,
      bodyVelocity=state.bodyVelocity,
      attitude=state.attitude,
      bodyAngularVelocity=state.bodyAngularVelocity),
    RigidBody.Wrench(
      bodyForce=wrench.bodyForce,
      bodyTorque=wrench.bodyTorque),
    RigidBody.Parameters(
      mass=parameters.mass,
      gravity=parameters.gravity,
      inertia=parameters.inertia,
      quaternionNormGain=parameters.quaternionNormGain));
end stateDerivative;
