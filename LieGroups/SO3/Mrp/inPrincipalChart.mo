within LieGroups.SO3.Mrp;
function inPrincipalChart
  "True away from the norm-one MRP shadow-switch boundary"
  input Real r[3];
  input Real margin(min=0.0) = 1.0e-6;
  output Boolean valid;
algorithm
  valid := margin < 1.0 and r * r < (1.0 - margin)^2;
end inPrincipalChart;
