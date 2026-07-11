within RigidBody;
function attitudeRate "Quaternion component of the rigid-body vector field"
  input RigidBody.State state;
  input RigidBody.Parameters parameters;
  output Real rate[4];
algorithm
  rate := LieGroups.SO3.Quat.kinematics(
    state.attitude, state.bodyAngularVelocity)
    - parameters.quaternionNormGain
      * (state.attitude * state.attitude - 1.0) * state.attitude;
end attitudeRate;
