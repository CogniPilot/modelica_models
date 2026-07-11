within Tests;
model LieGroupsTests "Core group identities, charts, Jacobians, and edge cases"
  constant Real tolerance = 1.0e-8;
  constant Real vectorTolerance = 5.0e-8;
  constant Real pi = 3.1415926535897932384626433832795;
  constant Real rotationVector[3] = {0.12, -0.08, 0.05};
  constant Real nearZeroVector[3] = {1.0e-12, -2.0e-12, 3.0e-12};
  constant Real se2Algebra[3] = {0.4, -0.2, 0.3};
  constant Real se3Algebra[6] = {0.4, -0.2, 0.3, 0.08, -0.04, 0.06};
  constant Real se23Algebra[9] = {
    0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04};
  constant Real euler[3] = {0.4, -0.3, 0.2};
  Real so2Identity;
  Real rotated2[2];
  Real se2Element[3];
  Real se2RoundTrip[3];
  Real se2Identity[3];
  Real quaternion[4];
  Real quaternionIdentity[4];
  Real rotationRoundTrip[3];
  Real nearZeroRoundTrip[3];
  Real R[3, 3];
  Real quaternionFromR[4];
  Real leftJacobian[3, 3];
  Real inverseLeftJacobian[3, 3];
  Real rightJacobian[3, 3];
  Real expectedRightJacobian[3, 3];
  Real mrp[3];
  Real mrpRoundTrip[3];
  Real eulerRoundTrip[3];
  Real se3Element[7];
  Real se3RoundTrip[6];
  Real se3Identity[7];
  Real se3AdjointIdentity[6, 6];
  Real se23Element[10];
  Real se23RoundTrip[9];
  Real se23Identity[10];
  Real se23AdjointIdentity[9, 9];
  Real mixed[10];
  Real saturated[3];
equation
  so2Identity = LieGroups.SO2.product(0.7, LieGroups.SO2.inverse(0.7));
  rotated2 = LieGroups.SO2.rotate(0.5 * pi, {1.0, 0.0});
  se2Element = LieGroups.SE2.exp_map(se2Algebra);
  se2RoundTrip = LieGroups.SE2.log_map(se2Element);
  se2Identity = LieGroups.SE2.product(se2Element, LieGroups.SE2.inverse(se2Element));

  quaternion = LieGroups.SO3.Quat.exp_map(rotationVector);
  quaternionIdentity = LieGroups.SO3.Quat.product(
    quaternion,
    LieGroups.SO3.Quat.inverse(quaternion));
  rotationRoundTrip = LieGroups.SO3.Quat.log_map(quaternion);
  nearZeroRoundTrip = LieGroups.SO3.Quat.log_map(
    LieGroups.SO3.Quat.exp_map(nearZeroVector));
  R = LieGroups.SO3.Quat.to_DCM(quaternion);
  quaternionFromR = LieGroups.SO3.Quat.from_DCM(R);
  leftJacobian = LieGroups.SO3.Quat.left_jacobian(rotationVector);
  inverseLeftJacobian = LieGroups.SO3.Quat.left_jacobian_inv(rotationVector);
  rightJacobian = LieGroups.SO3.Quat.right_jacobian(rotationVector);
  expectedRightJacobian = LieGroups.SO3.Quat.left_jacobian(-rotationVector);

  mrp = LieGroups.SO3.Mrp.exp_map(rotationVector);
  mrpRoundTrip = LieGroups.SO3.Mrp.log_map(mrp);
  eulerRoundTrip = LieGroups.SO3.EulerB321.from_Quat(
    LieGroups.SO3.EulerB321.to_Quat(euler));

  se3Element = LieGroups.SE3.Quat.exp_map(se3Algebra);
  se3RoundTrip = LieGroups.SE3.Quat.log_map(se3Element);
  se3Identity = LieGroups.SE3.Quat.product(
    se3Element,
    LieGroups.SE3.Quat.inverse(se3Element));
  se3AdjointIdentity = LieGroups.SE3.Quat.adjoint(se3Element)
    * LieGroups.SE3.Quat.adjoint(LieGroups.SE3.Quat.inverse(se3Element));

  se23Element = LieGroups.SE23.Quat.exp_map(se23Algebra);
  se23RoundTrip = LieGroups.SE23.Quat.log_map(se23Element);
  se23Identity = LieGroups.SE23.Quat.product(
    se23Element,
    LieGroups.SE23.Quat.inverse(se23Element));
  se23AdjointIdentity = LieGroups.SE23.Quat.adjoint(se23Element)
    * LieGroups.SE23.Quat.adjoint(LieGroups.SE23.Quat.inverse(se23Element));
  mixed = LieGroups.SE23.Quat.exp_mixed(
    {1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0},
    zeros(9),
    zeros(9),
    [0.0, 0.25; 0.0, 0.0]);
  saturated = LieGroups.Util.saturate3(
    {-2.0, 0.5, 3.0},
    {-1.0, -1.0, -1.0},
    {1.0, 1.0, 2.0});

  assert(abs(so2Identity) < tolerance, "SO(2) inverse identity failed");
  assert(Tests.Assertions.maxAbsVector(rotated2 - {0.0, 1.0}) < tolerance,
    "SO(2) quarter-turn rotation failed");
  assert(Tests.Assertions.maxAbsVector(se2RoundTrip - se2Algebra) < vectorTolerance,
    "SE(2) logarithm/exponential round trip failed");
  assert(Tests.Assertions.maxAbsVector(se2Identity) < tolerance,
    "SE(2) inverse identity failed");
  assert(abs(quaternionIdentity[1] - 1.0) < tolerance and
         Tests.Assertions.maxAbsVector(quaternionIdentity[2:4]) < tolerance,
    "Quaternion inverse identity failed");
  assert(Tests.Assertions.maxAbsVector(rotationRoundTrip - rotationVector) < vectorTolerance,
    "SO(3) logarithm/exponential round trip failed");
  assert(Tests.Assertions.maxAbsVector(nearZeroRoundTrip - nearZeroVector) < 1.0e-15,
    "SO(3) near-zero series failed");
  assert(Tests.Assertions.maxAbsMatrix(transpose(R) * R - identity(3)) < tolerance,
    "Quaternion DCM is not orthogonal");
  assert(abs(abs(quaternion * quaternionFromR) - 1.0) < tolerance,
    "DCM/quaternion round trip failed up to quaternion sign");
  assert(Tests.Assertions.maxAbsMatrix(leftJacobian * inverseLeftJacobian - identity(3)) < tolerance,
    "SO(3) left Jacobian inverse failed");
  assert(Tests.Assertions.maxAbsMatrix(rightJacobian - expectedRightJacobian) < tolerance,
    "SO(3) right/left Jacobian relation failed");
  assert(Tests.Assertions.maxAbsVector(mrpRoundTrip - rotationVector) < vectorTolerance,
    "MRP logarithm/exponential round trip failed");
  assert(Tests.Assertions.maxAbsVector(eulerRoundTrip - euler) < vectorTolerance,
    "Euler B321/quaternion round trip failed");
  assert(Tests.Assertions.maxAbsVector(se3RoundTrip - se3Algebra) < vectorTolerance,
    "SE(3) logarithm/exponential round trip failed");
  assert(Tests.Assertions.maxAbsVector(se3Identity - {0, 0, 0, 1, 0, 0, 0}) < tolerance,
    "SE(3) inverse identity failed");
  assert(Tests.Assertions.maxAbsMatrix(se3AdjointIdentity - identity(6)) < vectorTolerance,
    "SE(3) adjoint inverse identity failed");
  assert(Tests.Assertions.maxAbsVector(se23RoundTrip - se23Algebra) < vectorTolerance,
    "SE_2(3) logarithm/exponential round trip failed");
  assert(Tests.Assertions.maxAbsVector(se23Identity - {0, 0, 0, 0, 0, 0, 1, 0, 0, 0}) < tolerance,
    "SE_2(3) inverse identity failed");
  assert(Tests.Assertions.maxAbsMatrix(se23AdjointIdentity - identity(9)) < vectorTolerance,
    "SE_2(3) adjoint inverse identity failed");
  assert(Tests.Assertions.maxAbsVector(mixed[1:3] - {1.05, 1.975, -0.425}) < tolerance,
    "SE_2(3) mixed exponential coupling failed");
  assert(Tests.Assertions.maxAbsVector(saturated - {-1.0, 0.5, 2.0}) < tolerance,
    "Vector saturation failed");
  assert(abs(LieGroups.Util.angle_wrap(3.0 * pi) + pi) < tolerance,
    "Angle wrapping failed");
end LieGroupsTests;
