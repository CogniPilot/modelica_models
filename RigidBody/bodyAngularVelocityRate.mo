within RigidBody;
function bodyAngularVelocityRate
  "Body-angular-velocity component of the rigid-body vector field"
  input RigidBody.State state;
  input RigidBody.Wrench wrench;
  input RigidBody.Parameters parameters;
  output Real rate[3];
protected
  Real rightHandSide[3, 1];
  Real solution[3, 1];
  Boolean accepted;
algorithm
  rightHandSide[:, 1] := wrench.bodyTorque
    - cross(state.bodyAngularVelocity,
      parameters.inertia * state.bodyAngularVelocity);
  (solution, accepted) := LinearAlgebra.solveSPD(
    parameters.inertia, rightHandSide);
  assert(accepted, "Rigid-body inertia must be symmetric positive definite");
  rate := solution[:, 1];
end bodyAngularVelocityRate;
