within LieGroups.SO3.Quat;
function inverse_jacobian
  "Right-trivialized derivative of inverse with respect to its quaternion"
  input Real q[4] "Unit quaternion {w,x,y,z}";
  output Real J[3, 3] "d/d(dq) of log_map(inverse(q)^-1 * inverse(q*exp_map(dq)))";
algorithm
  J := -LieGroups.SO3.Quat.to_DCM(q);
  annotation(Documentation(info="<html>
    <p>Rule for <code>inverse</code>. Perturbing on the right and reading the
    inverted result back on the right gives</p>
    <pre>log_map(inverse(q)^-1 * inverse(q*exp_map(dq))) = -Ad(q) dq = -R(q) dq</pre>
    <p>The identity is exact: inversion turns the right perturbation into a left
    one and negates it, and the adjoint carries it back to the right.</p>
  </html>"));
end inverse_jacobian;
