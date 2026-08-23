within LieGroups.SE23.Quat;
function mixed_increment_coefficients
  "Rotation coefficients C1, C2, C3 of the mixed exponential increment block"
  input Real theta_sq "Squared magnitude of the algebra element's rotation part";
  output Real C[3] "{C1, C2, C3}";
protected
  Real theta;
  // Below 0.1 rad the retained series error is negligible, while the closed
  // coefficients lose their numerators to cancellation in single-precision
  // generated code. This is the same branch radius exp_mixed uses.
  constant Real eps = 1e-2;
algorithm
  if theta_sq < eps then
    C[1] := 0.5 - theta_sq / 24.0;
    C[2] := 1.0/6.0 - theta_sq / 120.0;
    C[3] := 1.0/24.0 - theta_sq / 720.0;
  else
    theta := sqrt(theta_sq);
    C[1] := (1.0 - cos(theta)) / theta_sq;
    C[2] := (theta - sin(theta)) / (theta_sq * theta);
    C[3] := (theta_sq/2.0 + cos(theta) - 1.0) / (theta_sq * theta_sq);
  end if;
  annotation(Documentation(info="<html>
    <p>C1 = (1-cos t)/t^2, C2 = (t-sin t)/t^3, C3 = (t^2/2 + cos t - 1)/t^4,
    with t^2 the input. These are the coefficients that appear in the increment
    block N of the closed-form mixed exponential; branch radius and retained
    series match <code>exp_mixed</code> term for term, so the two agree exactly.</p>
  </html>"));
end mixed_increment_coefficients;
