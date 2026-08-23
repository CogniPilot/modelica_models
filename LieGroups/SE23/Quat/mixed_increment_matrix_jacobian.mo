within LieGroups.SE23.Quat;
function mixed_increment_matrix_jacobian
  "Derivative of the mixed exponential increment block by its algebra element"
  input Real a[9] "Algebra element {vb, ab, omega}";
  input Real B[2, 2] "Coupling matrix as this increment sees it";
  output Real DN[6, 9]
    "Rows 1:3 are d N[:,1] / d a, rows 4:6 are d N[:,2] / d a";
protected
  Real omega[3];
  Real theta_sq;
  Real C[3];
  Real dC[3];
  Real Om[3, 3];
  Real Om2[3, 3];
  Real A[3, 2];
  Real I2[2, 2];
  Real M1[2, 2];
  Real M2[2, 2];
  Real M3[2, 2];
  Real AM2[3, 2];
  Real AM3[3, 2];
  Real OA[3, 2];
  Real O2A[3, 2];
  Real OAB[3, 2];
  Real O2AB[3, 2];
  Real gainVelocity[3];
  Real gainPosition[3];
algorithm
  omega := a[7:9];
  theta_sq := omega[1]^2 + omega[2]^2 + omega[3]^2;
  C := LieGroups.SE23.Quat.mixed_increment_coefficients(theta_sq);
  dC := LieGroups.SE23.Quat.mixed_increment_coefficient_derivatives(theta_sq);
  Om := LieGroups.SO3.Quat.wedge(omega);
  Om2 := Om * Om;
  A := {{a[4], a[1]}, {a[5], a[2]}, {a[6], a[3]}};
  I2 := identity(2);
  M1 := I2 + 0.5 * B;
  M2 := C[1] * I2 + C[2] * B;
  M3 := C[2] * I2 + C[3] * B;
  AM2 := A * M2;
  AM3 := A * M3;
  OA := Om * A;
  O2A := Om2 * A;
  OAB := OA * B;
  O2AB := O2A * B;

  // Acceleration slot a[4:6] is column 1 of A, velocity slot a[1:3] is column 2.
  DN[1:3, 4:6] := M1[1, 1] * identity(3) + M2[1, 1] * Om + M3[1, 1] * Om2;
  DN[1:3, 1:3] := M1[2, 1] * identity(3) + M2[2, 1] * Om + M3[2, 1] * Om2;
  DN[4:6, 4:6] := M1[1, 2] * identity(3) + M2[1, 2] * Om + M3[1, 2] * Om2;
  DN[4:6, 1:3] := M1[2, 2] * identity(3) + M2[2, 2] * Om + M3[2, 2] * Om2;

  // Rotation slot: skew factors carry d(Om)/d(omega), gain terms carry
  // d(C)/d(omega) through theta squared.
  DN[1:3, 7:9] := -LieGroups.SO3.Quat.wedge(AM2[:, 1])
    - LieGroups.SO3.Quat.wedge(Om * AM3[:, 1])
    - Om * LieGroups.SO3.Quat.wedge(AM3[:, 1]);
  DN[4:6, 7:9] := -LieGroups.SO3.Quat.wedge(AM2[:, 2])
    - LieGroups.SO3.Quat.wedge(Om * AM3[:, 2])
    - Om * LieGroups.SO3.Quat.wedge(AM3[:, 2]);
  gainVelocity := dC[1] * OA[:, 1] + dC[2] * OAB[:, 1]
    + dC[2] * O2A[:, 1] + dC[3] * O2AB[:, 1];
  gainPosition := dC[1] * OA[:, 2] + dC[2] * OAB[:, 2]
    + dC[2] * O2A[:, 2] + dC[3] * O2AB[:, 2];
  DN[1:3, 7:9] := DN[1:3, 7:9] + 2.0 * {
    {gainVelocity[1] * omega[1], gainVelocity[1] * omega[2], gainVelocity[1] * omega[3]},
    {gainVelocity[2] * omega[1], gainVelocity[2] * omega[2], gainVelocity[2] * omega[3]},
    {gainVelocity[3] * omega[1], gainVelocity[3] * omega[2], gainVelocity[3] * omega[3]}};
  DN[4:6, 7:9] := DN[4:6, 7:9] + 2.0 * {
    {gainPosition[1] * omega[1], gainPosition[1] * omega[2], gainPosition[1] * omega[3]},
    {gainPosition[2] * omega[1], gainPosition[2] * omega[2], gainPosition[2] * omega[3]},
    {gainPosition[3] * omega[1], gainPosition[3] * omega[2], gainPosition[3] * omega[3]}};
  annotation(Documentation(info="<html>
    <p>First variation of <code>mixed_increment_matrix</code> in its algebra
    slot, additive in that slot. With N = A M1 + Om A M2 + Om^2 A M3, where
    M1 = I + B/2, M2 = C1 I + C2 B and M3 = C2 I + C3 B, the A-slots contribute
    M1[j,k] I + M2[j,k] Om + M3[j,k] Om^2 to column k of N, and the rotation
    slot contributes the skew terms from d(Om) together with the coefficient
    terms 2 (dC/d(theta^2)) omega^T carried through theta squared.</p>
    <p>The packing is flat rather than three-dimensional so the rule composes by
    ordinary matrix products: rows 1:3 differentiate the velocity-slot column of
    N and rows 4:6 the position-slot column.</p>
  </html>"));
end mixed_increment_matrix_jacobian;
