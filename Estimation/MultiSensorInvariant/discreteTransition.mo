within Estimation.MultiSensorInvariant;

function discreteTransition
  "Third-order discretization of a held continuous tangent transition"
  input Real A[:, size(A, 1)];
  input Real dt(unit = "s");
  output Real transition[size(A, 1), size(A, 2)];
protected
  Real scaledA[size(A, 1), size(A, 2)];
  Real scaledA2[size(A, 1), size(A, 2)];
algorithm
  scaledA := A * dt;
  scaledA2 := scaledA * scaledA;
  transition := identity(size(A, 1)) + scaledA
    + 0.5 * scaledA2 + (1.0 / 6.0) * scaledA2 * scaledA;
end discreteTransition;
