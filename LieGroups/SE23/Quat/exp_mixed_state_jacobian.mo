within LieGroups.SE23.Quat;
function exp_mixed_state_jacobian
  "Right-trivialized derivative of exp_mixed with respect to its initial state"
  input Real X0[10] "Initial state {p0, v0, q0}";
  input Real l[9] "Left (body-frame) algebra element";
  input Real r[9] "Right (world-frame) algebra element";
  input Real B[2, 2] "Coupling matrix";
  output Real J[9, 9]
    "d/d(dX) of log_map(X1^-1 * exp_mixed(X0*exp_map(dX), l, r, B)), X1 = exp_mixed(X0, l, r, B)";
protected
  Real Nl[3, 2];
  Real RlTransposed[3, 3];
  Real M[2, 2];
algorithm
  Nl := LieGroups.SE23.Quat.mixed_increment_matrix(l, B);
  RlTransposed := transpose(LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.exp_map(l[7:9])));
  M := identity(2) + B;
  J := zeros(9, 9);
  J[1:3, 1:3] := M[2, 2] * RlTransposed;
  J[1:3, 4:6] := M[1, 2] * RlTransposed;
  J[1:3, 7:9] := -RlTransposed * LieGroups.SO3.Quat.wedge(Nl[:, 2]);
  J[4:6, 1:3] := M[2, 1] * RlTransposed;
  J[4:6, 4:6] := M[1, 1] * RlTransposed;
  J[4:6, 7:9] := -RlTransposed * LieGroups.SO3.Quat.wedge(Nl[:, 1]);
  J[7:9, 7:9] := RlTransposed;
  annotation(Documentation(info="<html>
    <p>Rule for <code>exp_mixed</code>, initial-state slot. The state is
    perturbed on the right, X0 -&gt; X0 exp_map(dX), and the result is read back
    on the right at X1 = exp_mixed(X0, l, r, B), so this returns J with</p>
    <pre>log_map(X1^-1 * exp_mixed(X0*exp_map(dX), l, r, B)) = J*dX + O(|dX|^2)</pre>
    <p>in the tangent ordering {position, velocity, rotation} that
    <code>exp_map</code> and <code>adjoint</code> use.</p>
    <p>Closed form. Writing R_l for the rotation of the left increment,
    N_l for its increment block and M = I + B, the state perturbation reaches
    the result only through the group product, so with the initial state's
    3-by-2 block perturbed by [dv | dp] and its rotation by d(omega):</p>
    <pre>[e_v | e_p] = R_l^T [dv | dp] M + R_l^T [d(omega)]x N_l
e_omega     = R_l^T d(omega)</pre>
    <p>Neither the right increment nor the initial state's own value appears:
    the right increment acts by left multiplication and drops out of a
    right-trivialized error, and the initial state cancels against its own
    inverse. The rotation coupling enters through the columns of N_l alone,
    which is what makes the whole 9-by-9 block closed form.</p>
  </html>"));
end exp_mixed_state_jacobian;
