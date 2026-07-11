within LieGroups.Rn;
function small_adjoint "R^n algebra adjoint"
  input Real tangent[:];
  output Real ad[size(tangent, 1), size(tangent, 1)];
algorithm
  ad := zeros(size(tangent, 1), size(tangent, 1));
end small_adjoint;
