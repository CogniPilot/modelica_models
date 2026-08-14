within Tests.LieGroupTests;
model NumericalEdgeCases "Zero, near-zero, pi, singular-chart, and threshold tests"
  constant Real pi = 3.1415926535897932384626433832795;
  Real zeroJl[3, 3];
  Real zeroJr[3, 3];
  Real zeroJlInv[3, 3];
  Real zeroJrInv[3, 3];
  Real nearJl1[3, 3];
  Real nearJl2[3, 3];
  Real nearJl3[3, 3];
  Real nearJl4[3, 3];
  Real nearJr4[3, 3];
  Real piQx[4];
  Real piQy[4];
  Real piQz[4];
  Real piLogX[3];
  Real piLogY[3];
  Real piLogZ[3];
  Real normalizedLogs[3, 3];
  Real mrpX[3];
  Real mrpY[3];
  Real mrpZ[3];
  Real mrpNear[3];
  Real mrpProducts[3, 3];
  Real qIdentity[4];
  Real qPiX[4];
  Real qPiY[4];
  Real qPiZ[4];
  Real qQuarterZ[4];
  Real se3QZero[3, 3];
  Real se3QTranslation[3, 3];
  Real se3QNearRotation[3, 3];
  Real se3QPiRotation[3, 3];
  Real se3RightQZero[3, 3];
  Real se3RightQTranslation[3, 3];
  Real se3RightQNearRotation[3, 3];
  Real se3RightQPiRotation[3, 3];
  Real se23PureNear[9, 9];
  Real se23PurePi[9, 9];
  Real se23PureLarge[9, 9];
  Real se23HoverLeft[9, 9];
  Real se23HoverRight[9, 9];
  Real thresholdJ1[3, 3];
  Real thresholdJ2[3, 3];
  Real thresholdJ3[3, 3];
  Real thresholdJ4[3, 3];
  Real thresholdJ5[3, 3];
  Real float32CriticalJl[3, 3];
  Real float32CriticalJr[3, 3];
  Real float32CriticalJlInv[3, 3];
  Real float32CriticalJrInv[3, 3];
  Real float32CriticalJlNegative[3, 3];
  Real float32CriticalQ[3, 3];
equation
  zeroJl = LieGroups.SO3.Quat.left_jacobian(zeros(3));
  zeroJr = LieGroups.SO3.Quat.right_jacobian(zeros(3));
  zeroJlInv = LieGroups.SO3.Quat.left_jacobian_inv(zeros(3));
  zeroJrInv = LieGroups.SO3.Quat.right_jacobian_inv(zeros(3));
  nearJl1 = LieGroups.SO3.Quat.left_jacobian({1.0e-10, 0.0, 0.0});
  nearJl2 = LieGroups.SO3.Quat.left_jacobian({0.0, 1.0e-10, 0.0});
  nearJl3 = LieGroups.SO3.Quat.left_jacobian({0.0, 0.0, 1.0e-10});
  nearJl4 = LieGroups.SO3.Quat.left_jacobian({1.0e-8, 1.0e-8, 1.0e-8});
  nearJr4 = LieGroups.SO3.Quat.right_jacobian({1.0e-8, 1.0e-8, 1.0e-8});
  piQx = LieGroups.SO3.Quat.exp_map({pi, 0.0, 0.0});
  piQy = LieGroups.SO3.Quat.exp_map({0.0, pi, 0.0});
  piQz = LieGroups.SO3.Quat.exp_map({0.0, 0.0, pi});
  piLogX = LieGroups.SO3.Quat.log_map(piQx);
  piLogY = LieGroups.SO3.Quat.log_map(piQy);
  piLogZ = LieGroups.SO3.Quat.log_map(piQz);
  normalizedLogs[1, :] = LieGroups.SO3.Quat.log_map({1.0, 0.0, 0.0, 0.0});
  normalizedLogs[2, :] = LieGroups.SO3.Quat.log_map({1.0e-10, 0.0, 0.0, 0.0});
  normalizedLogs[3, :] = LieGroups.SO3.Quat.log_map({0.0, 0.0, 0.0, 1.0});
  mrpX = LieGroups.SO3.Mrp.from_Quat({0.0, 1.0, 0.0, 0.0});
  mrpY = LieGroups.SO3.Mrp.from_Quat({0.0, 0.0, 1.0, 0.0});
  mrpZ = LieGroups.SO3.Mrp.from_Quat({0.0, 0.0, 0.0, 1.0});
  mrpNear = LieGroups.SO3.Mrp.from_Quat({-0.0001, 0.0, 0.0, 1.0});
  mrpProducts[1, :] = LieGroups.SO3.Mrp.product(zeros(3), zeros(3));
  mrpProducts[2, :] = LieGroups.SO3.Mrp.product({0.1, 0, 0}, {0.1, 0, 0});
  mrpProducts[3, :] = LieGroups.SO3.Mrp.product({0.5, 0, 0}, {-0.5, 0, 0});
  qIdentity = LieGroups.SO3.Quat.from_DCM(identity(3));
  qPiX = LieGroups.SO3.Quat.from_DCM([1, 0, 0; 0, -1, 0; 0, 0, -1]);
  qPiY = LieGroups.SO3.Quat.from_DCM([-1, 0, 0; 0, 1, 0; 0, 0, -1]);
  qPiZ = LieGroups.SO3.Quat.from_DCM([-1, 0, 0; 0, -1, 0; 0, 0, 1]);
  qQuarterZ = LieGroups.SO3.Quat.from_DCM([0, -1, 0; 1, 0, 0; 0, 0, 1]);
  se3QZero = LieGroups.SE3.Quat.left_Q(zeros(3), zeros(3));
  se3QTranslation = LieGroups.SE3.Quat.left_Q({1, 0, 0}, zeros(3));
  se3QNearRotation = LieGroups.SE3.Quat.left_Q(zeros(3), {1.0e-10, 0, 0});
  se3QPiRotation = LieGroups.SE3.Quat.left_Q({1, 2, 3}, {0, 0, pi});
  se3RightQZero = LieGroups.SE3.Quat.right_Q(zeros(3), zeros(3));
  se3RightQTranslation = LieGroups.SE3.Quat.right_Q({1, 0, 0}, zeros(3));
  se3RightQNearRotation = LieGroups.SE3.Quat.right_Q(zeros(3), {1.0e-10, 0, 0});
  se3RightQPiRotation = LieGroups.SE3.Quat.right_Q({1, 2, 3}, {0, 0, pi});
  se23PureNear = LieGroups.SE23.Quat.left_jacobian({0, 0, 0, 0, 0, 0, 1.0e-10, 0, 0});
  se23PurePi = LieGroups.SE23.Quat.left_jacobian({0, 0, 0, 0, 0, 0, 0, 0, pi});
  se23PureLarge = LieGroups.SE23.Quat.left_jacobian({0, 0, 0, 0, 0, 0, 1, 2, 3});
  se23HoverLeft = LieGroups.SE23.Quat.left_jacobian(
    {1.0e-8, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-10, 1.0e-10, 1.0e-10});
  se23HoverRight = LieGroups.SE23.Quat.right_jacobian(
    {1.0e-8, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-8, 1.0e-10, 1.0e-10, 1.0e-10});
  thresholdJ1 = LieGroups.SO3.Quat.left_jacobian({1.0e-6, 0, 0});
  thresholdJ2 = LieGroups.SO3.Quat.left_jacobian({sqrt(1.0e-7), 0, 0});
  thresholdJ3 = LieGroups.SO3.Quat.left_jacobian({sqrt(1.0e-5), 0, 0});
  thresholdJ4 = LieGroups.SO3.Quat.left_jacobian({sqrt(1.0e-9), 0, 0});
  thresholdJ5 = LieGroups.SO3.Quat.left_jacobian({sqrt(1.0e-6), 0, 0});
  // 1.67142628e-4 rad reproduces the first real RDD2 flight perturbation.
  // Its squared angle lies above the old 1e-8 cutoff, although Float32
  // cos(theta) rounds to one and makes the inverse closed form singular.
  float32CriticalJl = LieGroups.SO3.Quat.left_jacobian({1.67142628e-4, 0, 0});
  float32CriticalJr = LieGroups.SO3.Quat.right_jacobian({1.67142628e-4, 0, 0});
  float32CriticalJlInv = LieGroups.SO3.Quat.left_jacobian_inv({1.67142628e-4, 0, 0});
  float32CriticalJrInv = LieGroups.SO3.Quat.right_jacobian_inv({1.67142628e-4, 0, 0});
  float32CriticalJlNegative = LieGroups.SO3.Quat.left_jacobian({-1.67142628e-4, 0, 0});
  float32CriticalQ = LieGroups.SE3.Quat.left_Q(
    {0.0, 0.0, 0.18}, {1.67142628e-4, 0, 0});

  assert(Tests.Assertions.maxAbsMatrix(zeroJl - identity(3)) < 1.0e-12 and
         Tests.Assertions.maxAbsMatrix(zeroJr - identity(3)) < 1.0e-12 and
         Tests.Assertions.maxAbsMatrix(zeroJlInv - identity(3)) < 1.0e-12 and
         Tests.Assertions.maxAbsMatrix(zeroJrInv - identity(3)) < 1.0e-12,
    "SO3 Jacobians at zero failed");
  assert(Tests.Assertions.isFiniteMatrix(nearJl1) and
         Tests.Assertions.isFiniteMatrix(nearJl2) and
         Tests.Assertions.isFiniteMatrix(nearJl3) and
         Tests.Assertions.isFiniteMatrix(nearJl4) and
         Tests.Assertions.isFiniteMatrix(nearJr4),
    "SO3 Jacobian near-zero evaluation was not finite");
  assert(Tests.Assertions.isFiniteVector(piQx) and
         Tests.Assertions.isFiniteVector(piQy) and
         Tests.Assertions.isFiniteVector(piQz) and
         Tests.Assertions.isFiniteVector(piLogX) and
         Tests.Assertions.isFiniteVector(piLogY) and
         Tests.Assertions.isFiniteVector(piLogZ),
    "SO3 pi-rotation exp/log was not finite");
  assert(Tests.Assertions.isFiniteMatrix(normalizedLogs) and
         Tests.Assertions.isFiniteVector(mrpX) and
         Tests.Assertions.isFiniteVector(mrpY) and
         Tests.Assertions.isFiniteVector(mrpZ) and
         Tests.Assertions.isFiniteVector(mrpNear) and
         Tests.Assertions.isFiniteMatrix(mrpProducts),
    "Quaternion normalization or MRP critical-angle test was not finite");
  assert(Tests.Assertions.isFiniteVector(qIdentity) and
         Tests.Assertions.isFiniteVector(qPiX) and
         Tests.Assertions.isFiniteVector(qPiY) and
         Tests.Assertions.isFiniteVector(qPiZ) and
         Tests.Assertions.isFiniteVector(qQuarterZ),
    "DCM-to-quaternion critical-angle conversion was not finite");
  assert(Tests.Assertions.isFiniteMatrix(se3QZero) and
         Tests.Assertions.isFiniteMatrix(se3QTranslation) and
         Tests.Assertions.isFiniteMatrix(se3QNearRotation) and
         Tests.Assertions.isFiniteMatrix(se3QPiRotation) and
         Tests.Assertions.isFiniteMatrix(se3RightQZero) and
         Tests.Assertions.isFiniteMatrix(se3RightQTranslation) and
         Tests.Assertions.isFiniteMatrix(se3RightQNearRotation) and
         Tests.Assertions.isFiniteMatrix(se3RightQPiRotation),
    "SE3 Q-block critical-point evaluation was not finite");
  assert(Tests.Assertions.isFiniteMatrix(se23PureNear) and
         Tests.Assertions.isFiniteMatrix(se23PurePi) and
         Tests.Assertions.isFiniteMatrix(se23PureLarge) and
         Tests.Assertions.isFiniteMatrix(se23HoverLeft) and
         Tests.Assertions.isFiniteMatrix(se23HoverRight),
    "SE_2(3) critical-state Jacobian was not finite");
  assert(Tests.Assertions.isFiniteMatrix(thresholdJ1) and
         Tests.Assertions.isFiniteMatrix(thresholdJ2) and
         Tests.Assertions.isFiniteMatrix(thresholdJ3) and
         Tests.Assertions.isFiniteMatrix(thresholdJ4) and
         Tests.Assertions.isFiniteMatrix(thresholdJ5),
    "Taylor-series threshold evaluation was not finite");
  assert(Tests.Assertions.isFiniteMatrix(float32CriticalJl) and
         Tests.Assertions.isFiniteMatrix(float32CriticalJr) and
         Tests.Assertions.isFiniteMatrix(float32CriticalJlInv) and
         Tests.Assertions.isFiniteMatrix(float32CriticalJrInv) and
         Tests.Assertions.maxAbsMatrix(
           float32CriticalJl * float32CriticalJlInv - identity(3)) < 1.0e-10 and
         Tests.Assertions.maxAbsMatrix(
           float32CriticalJr * float32CriticalJrInv - identity(3)) < 1.0e-10 and
         Tests.Assertions.maxAbsMatrix(
           float32CriticalJr - float32CriticalJlNegative) < 1.0e-12 and
         Tests.Assertions.isFiniteMatrix(float32CriticalQ),
    "Float32-critical Lie-group Jacobian evaluation was not finite");
end NumericalEdgeCases;
