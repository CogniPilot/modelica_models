within LieGroups.SE23.Quat;
function inverse_jacobian
  "Right-trivialized derivative of inverse with respect to its element"
  input Real X[10] "Element {p, v, q}";
  output Real J[9, 9]
    "d/d(dX) of log_map(inverse(X)^-1 * inverse(X*exp_map(dX)))";
algorithm
  J := -LieGroups.SE23.Quat.adjoint(X);
  annotation(Documentation(info="<html>
    <p>Rule for <code>inverse</code>:</p>
    <pre>log_map(inverse(X)^-1 * inverse(X*exp_map(dX))) = -Ad(X) dX</pre>
    <p>The identity is exact. Inversion turns the right perturbation into a left
    one and negates it; the adjoint carries it back to the right.</p>
  </html>"));
end inverse_jacobian;
