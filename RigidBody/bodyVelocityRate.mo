within RigidBody;
function bodyVelocityRate "Body-velocity component of the rigid-body vector field"
  input RigidBody.State state;
  input RigidBody.Wrench wrench;
  input RigidBody.Parameters parameters;
  output Real rate[3];
protected
  Real gravityBody[3];
algorithm
  assert(parameters.mass > 0.0, "Rigid-body mass must be positive");
  gravityBody := transpose(LieGroups.SO3.Quat.to_DCM(state.attitude))
    * {0.0, 0.0, -parameters.gravity};
  rate := wrench.bodyForce / parameters.mass + gravityBody
    - cross(state.bodyAngularVelocity, state.bodyVelocity);
end bodyVelocityRate;
