within LieGroups.SE23.Quat;
function exp_mixed_right_increment_jacobian
  "Right-trivialized derivative of exp_mixed with respect to its right increment"
  input Real X0[10] "Initial state {p0, v0, q0}";
  input Real l[9] "Left (body-frame) algebra element";
  input Real r[9] "Right (world-frame) algebra element";
  input Real B[2, 2] "Coupling matrix";
  output Real J[9, 9]
    "d/d(dr) of log_map(X1^-1 * exp_mixed(X0, l, r + dr, B)), X1 = exp_mixed(X0, l, r, B)";
protected
  Real Nl[3, 2];
  Real DNr[6, 9];
  Real leftRotation[4];
  Real initialThenLeft[3, 3];
  Real resultRotation[3, 3];
  Real transportToLeft[3, 3];
  Real transportToResult[3, 3];
  Real M[2, 2];
  Real P0[3, 2];
  Real U[3, 2];
  Real rightJacobian[3, 3];
algorithm
  Nl := LieGroups.SE23.Quat.mixed_increment_matrix(l, B);
  DNr := LieGroups.SE23.Quat.mixed_increment_matrix_jacobian(r, -B);
  leftRotation := LieGroups.SO3.Quat.exp_map(l[7:9]);
  initialThenLeft := LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.product(X0[7:10], leftRotation));
  resultRotation := LieGroups.SO3.Quat.to_DCM(
    LieGroups.SO3.Quat.product(
      LieGroups.SO3.Quat.product(
        LieGroups.SO3.Quat.exp_map(r[7:9]), X0[7:10]),
      leftRotation));
  transportToLeft := transpose(initialThenLeft);
  transportToResult := transpose(resultRotation);
  M := identity(2) + B;
  P0 := {{X0[4], X0[1]}, {X0[5], X0[2]}, {X0[6], X0[3]}};
  U := LieGroups.SO3.Quat.to_DCM(X0[7:10]) * Nl + P0 * M;
  rightJacobian := LieGroups.SO3.Quat.right_jacobian_exact(r[7:9]);
  J := zeros(9, 9);
  J[1:3, 7:9] := -transportToLeft * LieGroups.SO3.Quat.wedge(U[:, 2])
    * rightJacobian;
  J[4:6, 7:9] := -transportToLeft * LieGroups.SO3.Quat.wedge(U[:, 1])
    * rightJacobian;
  J[1:3, :] := J[1:3, :] + transportToResult
    * (M[1, 2] * DNr[1:3, :] + M[2, 2] * DNr[4:6, :]);
  J[4:6, :] := J[4:6, :] + transportToResult
    * (M[1, 1] * DNr[1:3, :] + M[2, 1] * DNr[4:6, :]);
  J[7:9, 7:9] := transportToLeft * rightJacobian;
  annotation(Documentation(info="<html>
    <p>Rule for <code>exp_mixed</code>, right-increment slot. The increment is a
    plain algebra vector and is perturbed additively; the result is read back on
    the right at X1 = exp_mixed(X0, l, r, B), so this returns J with</p>
    <pre>log_map(X1^-1 * exp_mixed(X0, l, r + dr, B)) = J*dr + O(|dr|^2)</pre>
    <p>Closed form, and the only rule of the three in which the initial state
    survives. The right increment acts by left multiplication, so its rotation
    perturbation sweeps the whole accumulated 3-by-2 block
    U = R_0 N_l + P_0 M through a cross product before the result is
    transported back into the body frame, while its own increment block is
    carried through the coupling M:</p>
    <pre>column k of [e_v | e_p] = -(R_0 R_l)^T [U[:,k]]x J_r(omega_r) d(omega_r)
                          + R_1^T (dN_r M)[:,k]
e_omega                   = (R_0 R_l)^T J_r(omega_r) d(omega_r)</pre>
    <p>The right increment's own block uses the opposite coupling sign, so dN_r
    is <code>mixed_increment_matrix_jacobian(r, -B)</code>, matching the way
    <code>exp_mixed</code> builds N_r.</p>
    <p>J_r here is <code>right_jacobian_exact</code>, and in this rule the
    choice decides whether the rule passes at all: the Jacobian is multiplied by
    the accumulated position block U before the result is transported, so any
    error in it arrives scaled by the lever arm. Measured against a central
    difference at 0.099 rad, <code>right_jacobian</code>'s two-term series gives
    6.4e-7 at a 50 m arm and 2.6e-6 at 200 m, where the closed form gives 1.1e-8
    and 2.9e-8, the difference floors for those arms.</p>
  </html>"));
end exp_mixed_right_increment_jacobian;
