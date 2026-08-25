within LieGroups.SE23.Quat;
function mixed_increment_coefficient_derivatives
  "Derivatives of the mixed exponential increment coefficients by theta squared"
  input Real theta_sq "Squared magnitude of the algebra element's rotation part";
  output Real dC[3] "d{C1, C2, C3} / d(theta_sq)";
protected
  Real C[3];
  constant Real eps = 1e-2;
algorithm
  if theta_sq < eps then
    // Exact derivative of the retained series mixed_increment_coefficients
    // evaluates on this branch, so rule and primitive stay consistent.
    dC[1] := -1.0/24.0;
    dC[2] := -1.0/120.0;
    dC[3] := -1.0/720.0;
  else
    // Written in the coefficients themselves rather than in sines and cosines.
    // The direct forms subtract quantities of order theta^2 to leave numerators
    // of order theta^4, theta^5, and theta^6; expressed this way the worst
    // cancellation left is of order theta^2/60, which single-precision
    // generated code can carry down to the branch radius.
    C := LieGroups.SE23.Quat.mixed_increment_coefficients(theta_sq);
    dC[1] := C[3] - 0.5 * C[2];
    dC[2] := (C[1] - 3.0 * C[2]) / (2.0 * theta_sq);
    dC[3] := (0.5 * C[2] - 2.0 * C[3]) / theta_sq;
  end if;
  annotation(Documentation(info="<html>
    <p>With s = theta^2 and C1, C2, C3 as in
    <code>mixed_increment_coefficients</code>, the identities
    1 - cos t = C1 s and t - sin t = C2 s t give C3 = (1/2 - C1)/s, and
    differentiating that relation yields</p>
    <pre>dC1/ds = C3 - C2/2
dC2/ds = (C1 - 3 C2) / (2 s)
dC3/ds = (C2/2 - 2 C3) / s</pre>
    <p>which is what this function evaluates above the branch radius.</p>
  </html>"));
end mixed_increment_coefficient_derivatives;
