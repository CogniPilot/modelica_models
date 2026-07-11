within LieGroups.Analysis;
function logQuadraticPotential
  "Local quadratic Lyapunov candidate in principal SO(3) logarithm coordinates"
  input Real R[3, 3];
  input Real metric[3, 3] = identity(3);
  output Real value;
protected
  Real tangent[3];
algorithm
  tangent := LieGroups.SO3.Dcm.log_map(R);
  value := 0.5 * LinearAlgebra.quadraticForm(tangent, metric);
end logQuadraticPotential;
