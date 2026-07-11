within LieGroups.Analysis;
function attitudePotential
  "Sign- and coordinate-independent SO(3) potential 1-cos(angle)"
  input Real R[3, 3];
  output Real value;
algorithm
  value := 0.5 * (3.0 - R[1, 1] - R[2, 2] - R[3, 3]);
end attitudePotential;
