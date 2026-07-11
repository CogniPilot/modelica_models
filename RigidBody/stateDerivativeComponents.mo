within RigidBody;
function stateDerivativeComponents
  "Array outputs of the pure rigid-body vector field for equation-based models"
  input RigidBody.State state;
  input RigidBody.Wrench wrench;
  input RigidBody.Parameters parameters;
  output Real worldPositionRate[3];
  output Real bodyVelocityRate[3];
  output Real attitudeRate[4];
  output Real bodyAngularVelocityRate[3];
algorithm
  assert(RigidBody.validParameters(parameters),
    "Rigid-body vector field requires positive mass and symmetric positive-definite inertia");
  worldPositionRate := RigidBody.worldPositionRate(state);
  bodyVelocityRate := RigidBody.bodyVelocityRate(state, wrench, parameters);
  attitudeRate := RigidBody.attitudeRate(state, parameters);
  bodyAngularVelocityRate := RigidBody.bodyAngularVelocityRate(
    state, wrench, parameters);
end stateDerivativeComponents;
