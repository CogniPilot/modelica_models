within LieGroups.Analysis;
function quadraticTangentPotential
  "Dimension-generic quadratic candidate on any Lie algebra coordinate vector"
  input Real tangent[:];
  input Real metric[size(tangent, 1), size(tangent, 1)];
  output Real value;
algorithm
  value := 0.5 * LinearAlgebra.quadraticForm(tangent, metric);
end quadraticTangentPotential;
