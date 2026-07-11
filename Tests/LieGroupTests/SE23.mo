within Tests.LieGroupTests;
model SE23 "SE_2(3) algebra, group, adjoint, and Jacobian tests"
  constant Real tolerance = 8.0e-8;
  constant Real tangent[9] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
  Real wedge[5, 5];
  Real vee[9];
  Real element[10];
  Real identityProduct[10];
  Real inverseIdentity[10];
  Real roundTrip[9];
  Real matrixRepresentation[5, 5];
  Real Ad[9, 9];
  Real ad[9, 9];
  Real Jl[9, 9];
  Real Jr[9, 9];
  Real JlInv[9, 9];
  Real JrInv[9, 9];
  Real zeroJl[9, 9];
  Real zeroJr[9, 9];
equation
  wedge = LieGroups.SE23.Quat.wedge(tangent);
  vee = LieGroups.SE23.Quat.vee(wedge);
  element = LieGroups.SE23.Quat.exp_map(tangent);
  identityProduct = LieGroups.SE23.Quat.product(
    element, {0, 0, 0, 0, 0, 0, 1, 0, 0, 0});
  inverseIdentity = LieGroups.SE23.Quat.product(
    element, LieGroups.SE23.Quat.inverse(element));
  roundTrip = LieGroups.SE23.Quat.log_map(element);
  matrixRepresentation = LieGroups.SE23.Quat.to_Matrix(element);
  Ad = LieGroups.SE23.Quat.adjoint(element);
  ad = LieGroups.SE23.Quat.small_adjoint(tangent);
  Jl = LieGroups.SE23.Quat.left_jacobian(tangent);
  Jr = LieGroups.SE23.Quat.right_jacobian(tangent);
  JlInv = LieGroups.SE23.Quat.left_jacobian_inv(tangent);
  JrInv = LieGroups.SE23.Quat.right_jacobian_inv(tangent);
  zeroJl = LieGroups.SE23.Quat.left_jacobian(zeros(9));
  zeroJr = LieGroups.SE23.Quat.right_jacobian(zeros(9));

  assert(Tests.Assertions.maxAbsVector(vee - tangent) < tolerance,
    "se_2(3) wedge/vee failed");
  assert(Tests.Assertions.maxAbsVector(identityProduct - element) < tolerance,
    "SE_2(3) identity product failed");
  assert(Tests.Assertions.maxAbsVector(
      inverseIdentity - {0, 0, 0, 0, 0, 0, 1, 0, 0, 0}) < tolerance,
    "SE_2(3) inverse failed");
  assert(Tests.Assertions.maxAbsVector(roundTrip - tangent) < tolerance,
    "SE_2(3) exp/log failed");
  assert(Tests.Assertions.isFiniteMatrix(matrixRepresentation) and
         Tests.Assertions.isFiniteMatrix(Ad) and
         Tests.Assertions.isFiniteMatrix(ad),
    "SE_2(3) matrix or adjoint construction failed");
  assert(Tests.Assertions.maxAbsMatrix(Jl * JlInv - identity(9)) < tolerance and
         Tests.Assertions.maxAbsMatrix(Jr * JrInv - identity(9)) < tolerance,
    "SE_2(3) Jacobian inverse tests failed");
  assert(Tests.Assertions.maxAbsMatrix(Jl - Ad * Jr) < tolerance and
         Tests.Assertions.maxAbsMatrix(JrInv - Ad * JlInv) < tolerance,
    "SE_2(3) adjoint/Jacobian identities failed");
  assert(Tests.Assertions.maxAbsMatrix(zeroJl - identity(9)) < tolerance and
         Tests.Assertions.maxAbsMatrix(zeroJr - identity(9)) < tolerance,
    "SE_2(3) Jacobians at zero failed");
end SE23;
