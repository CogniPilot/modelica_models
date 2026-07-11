within RigidBody;
function validParameters
  "Physical-domain predicate for mass, gravity, and symmetric positive-definite inertia"
  input RigidBody.Parameters parameters;
  output Boolean valid;
protected
  Real J[3, 3];
  Real determinant;
algorithm
  J := parameters.inertia;
  determinant := J[1, 1] * J[2, 2] * J[3, 3]
    + 2.0 * J[1, 2] * J[1, 3] * J[2, 3]
    - J[1, 1] * J[2, 3]^2 - J[2, 2] * J[1, 3]^2
    - J[3, 3] * J[1, 2]^2;
  valid := parameters.mass > 0.0 and parameters.gravity >= 0.0
    and abs(J[1, 2] - J[2, 1]) < 1.0e-12
    and abs(J[1, 3] - J[3, 1]) < 1.0e-12
    and abs(J[2, 3] - J[3, 2]) < 1.0e-12
    and J[1, 1] > 0.0
    and J[1, 1] * J[2, 2] - J[1, 2]^2 > 0.0
    and determinant > 0.0;
end validParameters;
