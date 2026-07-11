within Tests.LieGroupTests;
model SO3 "SO(3) algebra, representations, Jacobians, and conversions"
  constant Real tolerance = 2.0e-8;
  constant Real tangent[3] = {0.1, 0.2, 0.3};
  constant Real euler[3] = {0.3, -0.2, 0.4};
  Real wedge[3, 3];
  Real quaternion[4];
  Real quaternionIdentity[4];
  Real quaternionRoundTrip[4];
  Real mrp[3];
  Real mrpIdentity[3];
  Real mrpRoundTrip[3];
  Real R[3, 3];
  Real dcmIdentity[3, 3];
  Real dcmRoundTrip[3, 3];
  Real eulerIdentity[3];
  Real eulerRoundTrip[3];
  Real Jl[3, 3];
  Real Jr[3, 3];
  Real JlInv[3, 3];
  Real JrInv[3, 3];
  Real AdExp[3, 3];
  Real eulerJr[3, 3];
  Real eulerJl[3, 3];
  Real negativeQuaternionR[3, 3];
  LieGroups.SO3.EulerSequences.B232.Orientation B232Element;
  LieGroups.SO3.EulerSequences.S123.Orientation S123Element;
  Real B232R[3, 3];
  Real S123R[3, 3];
equation
  wedge = LieGroups.SO3.Quat.wedge(tangent);
  quaternion = LieGroups.SO3.Quat.exp_map(tangent);
  quaternionIdentity = LieGroups.SO3.Quat.product(
    quaternion, LieGroups.SO3.Quat.inverse(quaternion));
  quaternionRoundTrip = LieGroups.SO3.Quat.exp_map(
    LieGroups.SO3.Quat.log_map(quaternion));
  mrp = LieGroups.SO3.Mrp.exp_map(tangent);
  mrpIdentity = LieGroups.SO3.Mrp.product(mrp, LieGroups.SO3.Mrp.inverse(mrp));
  mrpRoundTrip = LieGroups.SO3.Mrp.exp_map(LieGroups.SO3.Mrp.log_map(mrp));
  R = LieGroups.SO3.Dcm.exp_map(tangent);
  dcmIdentity = LieGroups.SO3.Dcm.product(R, LieGroups.SO3.Dcm.inverse(R));
  dcmRoundTrip = LieGroups.SO3.Dcm.exp_map(LieGroups.SO3.Dcm.log_map(R));
  eulerIdentity = LieGroups.SO3.EulerB321.product(
    euler, LieGroups.SO3.EulerB321.inverse(euler));
  eulerRoundTrip = LieGroups.SO3.EulerB321.exp_map(
    LieGroups.SO3.EulerB321.log_map(euler));
  Jl = LieGroups.SO3.Quat.left_jacobian(tangent);
  Jr = LieGroups.SO3.Quat.right_jacobian(tangent);
  JlInv = LieGroups.SO3.Quat.left_jacobian_inv(tangent);
  JrInv = LieGroups.SO3.Quat.right_jacobian_inv(tangent);
  AdExp = LieGroups.SO3.Quat.adjoint(quaternion);
  eulerJr = LieGroups.SO3.EulerB321.right_jacobian(euler);
  eulerJl = LieGroups.SO3.EulerB321.left_jacobian(euler);
  negativeQuaternionR = LieGroups.SO3.Quat.to_DCM(-quaternion);
  B232Element = LieGroups.SO3.EulerSequences.B232.from_Matrix(R);
  S123Element = LieGroups.SO3.EulerSequences.S123.from_Matrix(R);
  B232R = LieGroups.SO3.EulerSequences.B232.to_Matrix(B232Element);
  S123R = LieGroups.SO3.EulerSequences.S123.to_Matrix(S123Element);

  assert(Tests.Assertions.maxAbsVector(LieGroups.SO3.Quat.vee(wedge) - tangent) < tolerance,
    "so3 wedge/vee failed");
  assert(Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.Quat.small_adjoint(tangent) - wedge) < tolerance,
    "so3 adjoint failed");
  assert(abs(quaternionIdentity[1] - 1.0) < tolerance and
         Tests.Assertions.maxAbsVector(quaternionIdentity[2:4]) < tolerance,
    "SO3 quaternion inverse failed");
  assert(abs(abs(quaternionRoundTrip * quaternion) - 1.0) < tolerance,
    "SO3 quaternion exp/log failed");
  assert(Tests.Assertions.maxAbsVector(mrpIdentity) < tolerance and
         Tests.Assertions.maxAbsVector(mrpRoundTrip - mrp) < tolerance,
    "SO3 MRP identity or exp/log failed");
  assert(Tests.Assertions.maxAbsMatrix(dcmIdentity - identity(3)) < tolerance and
         Tests.Assertions.maxAbsMatrix(dcmRoundTrip - R) < tolerance,
    "SO3 DCM identity or exp/log failed");
  assert(Tests.Assertions.maxAbsVector(eulerIdentity) < tolerance and
         Tests.Assertions.maxAbsMatrix(
           LieGroups.SO3.EulerB321.to_DCM(eulerRoundTrip)
             - LieGroups.SO3.EulerB321.to_DCM(euler)) < tolerance,
    "SO3 Euler identity or exp/log failed");
  assert(Tests.Assertions.maxAbsMatrix(Jl * JlInv - identity(3)) < tolerance and
         Tests.Assertions.maxAbsMatrix(Jr * JrInv - identity(3)) < tolerance,
    "SO3 Jacobian inverse tests failed");
  assert(Tests.Assertions.maxAbsMatrix(Jl - AdExp * Jr) < tolerance and
         Tests.Assertions.maxAbsMatrix(JrInv - AdExp * JlInv) < tolerance,
    "SO3 adjoint/Jacobian identities failed");
  assert(Tests.Assertions.maxAbsMatrix(
      eulerJl - eulerJr * LieGroups.SO3.EulerB321.to_DCM(euler)) < tolerance,
    "Euler left/right Jacobian consistency failed");
  assert(Tests.Assertions.maxAbsMatrix(
      LieGroups.SO3.EulerB321.right_jacobian(zeros(3))
        - [0, 0, 1; 0, 1, 0; 1, 0, 0]) < tolerance,
    "Euler Jacobian-at-zero test failed");
  assert(Tests.Assertions.maxAbsMatrix(negativeQuaternionR - R) < tolerance,
    "Negative-quaternion DCM test failed");
  assert(Tests.Assertions.maxAbsMatrix(B232R - R) < tolerance and
         Tests.Assertions.maxAbsMatrix(S123R - R) < tolerance,
    "Named B232 or S123 Euler representation failed");
  assert(Tests.LieGroupTests.allEulerSequencesPass(),
    "One or more of the 24 Euler sequence/convention forms failed");
  assert(Tests.Assertions.maxAbsMatrix(transpose(R) * R - identity(3)) < tolerance and
         abs(Tests.Assertions.determinant3(R) - 1.0) < tolerance,
    "DCM orthogonality/determinant tests failed");
end SO3;
