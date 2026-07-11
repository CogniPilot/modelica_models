within RigidBody;
function powerBalanceResidual
  "Mechanical-energy rate minus wrench supply; zero certifies lossless passivity"
  input RigidBody.State state;
  input RigidBody.StateDerivative derivative;
  input RigidBody.Wrench wrench;
  input RigidBody.Parameters parameters;
  output Real residual;
algorithm
  residual := parameters.mass * state.bodyVelocity * derivative.bodyVelocity
    + state.bodyAngularVelocity
      * (parameters.inertia * derivative.bodyAngularVelocity)
    + parameters.mass * parameters.gravity * derivative.worldPosition[3]
    - RigidBody.wrenchPower(state, wrench);
end powerBalanceResidual;
