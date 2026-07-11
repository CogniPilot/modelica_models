within LieGroups.SE23.Quat;
function left_jacobian "Forward left Jacobian J_l of SE_2(3) (9x9)"
  input Real xi[9] "Lie algebra {vb, ab, omega}";
  output Real J[9,9] "9x9 left Jacobian";
protected
  Real Jl[3,3] "SO(3) left Jacobian of omega";
  Real Qv[3,3] "Q block for velocity-column part vb";
  Real Qa[3,3] "Q block for acceleration-column part ab";
  Real omega[3];
algorithm
  omega := xi[7:9];
  Jl := LieGroups.SO3.Quat.left_jacobian(omega);
  Qv := LieGroups.SE3.Quat.left_Q(xi[1:3], omega);
  Qa := LieGroups.SE3.Quat.left_Q(xi[4:6], omega);

  // Block structure (ordering {vb, ab, omega}):
  // J = {{Jl, 0, Qv}, {0, Jl, Qa}, {0, 0, Jl}}
  J := zeros(9, 9);
  J[1:3, 1:3] := Jl;
  J[1:3, 7:9] := Qv;
  J[4:6, 4:6] := Jl;
  J[4:6, 7:9] := Qa;
  J[7:9, 7:9] := Jl;

  annotation(Documentation(info="<html>
    <p>Forward left Jacobian of SE_2(3) for the {vb, ab, omega} tangent ordering.</p>
    <p>Used by log-linear dynamic-inversion control: n = nbar - J_l(xi) BK xi.</p>
  </html>"));
end left_jacobian;
