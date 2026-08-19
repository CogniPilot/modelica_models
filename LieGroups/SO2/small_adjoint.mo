within LieGroups.SO2;
function small_adjoint "so(2) adjoint"
  input Real omega;
  output Real ad[1, 1];
algorithm
  ad[1, 1] := 0.0;
end small_adjoint;
