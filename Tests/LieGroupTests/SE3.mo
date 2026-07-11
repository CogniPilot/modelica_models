within Tests.LieGroupTests;
model SE3 "SE(3) algebra, group, Jacobian, and Q-block tests"
  constant Real tolerance = 5.0e-8;
  constant Real tangent[6] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  Real wedge[4, 4];
  Real vee[6];
  Real element[7];
  Real identityProduct[7];
  Real inverseIdentity[7];
  Real roundTrip[6];
  Real matrixRepresentation[4, 4];
  Real Ad[6, 6];
  Real ad[6, 6];
  Real Jl[6, 6];
  Real Jr[6, 6];
  Real JlInv[6, 6];
  Real JrInv[6, 6];
  Real leftQ[3, 3];
  Real rightQ[3, 3];
  Real zeroJl[6, 6];
  Real zeroJr[6, 6];
  Real piQ[3, 3];
equation
  wedge = LieGroups.SE3.Quat.wedge(tangent);
  vee = LieGroups.SE3.Quat.vee(wedge);
  element = LieGroups.SE3.Quat.exp_map(tangent);
  identityProduct = LieGroups.SE3.Quat.product(
    element, {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  inverseIdentity = LieGroups.SE3.Quat.product(
    element, LieGroups.SE3.Quat.inverse(element));
  roundTrip = LieGroups.SE3.Quat.log_map(element);
  matrixRepresentation = LieGroups.SE3.Quat.to_Matrix(element);
  Ad = LieGroups.SE3.Quat.adjoint(element);
  ad = LieGroups.SE3.Quat.small_adjoint(tangent);
  Jl = LieGroups.SE3.Quat.left_jacobian(tangent);
  Jr = LieGroups.SE3.Quat.right_jacobian(tangent);
  JlInv = LieGroups.SE3.Quat.left_jacobian_inv(tangent);
  JrInv = LieGroups.SE3.Quat.right_jacobian_inv(tangent);
  leftQ = LieGroups.SE3.Quat.left_Q(tangent[1:3], tangent[4:6]);
  rightQ = LieGroups.SE3.Quat.right_Q(tangent[1:3], tangent[4:6]);
  zeroJl = LieGroups.SE3.Quat.left_jacobian(zeros(6));
  zeroJr = LieGroups.SE3.Quat.right_jacobian(zeros(6));
  piQ = LieGroups.SE3.Quat.left_Q({1.0, 2.0, 3.0}, {0.0, 0.0, 3.141592653589793});

  assert(Tests.Assertions.maxAbsVector(vee - tangent) < tolerance,
    "se3 wedge/vee failed");
  assert(Tests.Assertions.maxAbsVector(identityProduct - element) < tolerance,
    "SE3 identity product failed");
  assert(Tests.Assertions.maxAbsVector(
      inverseIdentity - {0, 0, 0, 1, 0, 0, 0}) < tolerance,
    "SE3 inverse failed");
  assert(Tests.Assertions.maxAbsVector(roundTrip - tangent) < tolerance,
    "SE3 exp/log failed");
  assert(Tests.Assertions.isFiniteMatrix(matrixRepresentation) and
         Tests.Assertions.isFiniteMatrix(Ad) and
         Tests.Assertions.isFiniteMatrix(ad),
    "SE3 matrix or adjoint construction failed");
  assert(Tests.Assertions.maxAbsMatrix(Jl * JlInv - identity(6)) < tolerance and
         Tests.Assertions.maxAbsMatrix(Jr * JrInv - identity(6)) < tolerance,
    "SE3 Jacobian inverse tests failed");
  assert(Tests.Assertions.maxAbsMatrix(Jl - Ad * Jr) < tolerance and
         Tests.Assertions.maxAbsMatrix(JrInv - Ad * JlInv) < tolerance,
    "SE3 adjoint/Jacobian identities failed");
  assert(Tests.Assertions.maxAbsMatrix(zeroJl - identity(6)) < tolerance and
         Tests.Assertions.maxAbsMatrix(zeroJr - identity(6)) < tolerance,
    "SE3 Jacobians at zero failed");
  assert(Tests.Assertions.isFiniteMatrix(leftQ) and
         Tests.Assertions.isFiniteMatrix(rightQ) and
         Tests.Assertions.isFiniteMatrix(piQ),
    "SE3 Q-block evaluation was not finite");
end SE3;
