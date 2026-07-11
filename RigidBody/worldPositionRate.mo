within RigidBody;
function worldPositionRate "World-position component of the rigid-body vector field"
  input RigidBody.State state;
  output Real rate[3];
algorithm
  rate := LieGroups.SO3.Quat.to_DCM(state.attitude) * state.bodyVelocity;
end worldPositionRate;
