within LieGroups.Analysis;
function inPrincipalLogDomain
  "True when an SO(3) matrix stays a requested margin away from the log cut locus"
  input Real R[3, 3];
  input Real margin(min=0.0) = 1.0e-6;
  output Boolean valid;
protected
  constant Real pi = 2.0 * asin(1.0);
algorithm
  valid := margin < pi and
    LieGroups.Analysis.rotationAngle(R) < pi - margin;
end inPrincipalLogDomain;
