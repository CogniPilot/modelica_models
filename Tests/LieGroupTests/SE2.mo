within Tests.LieGroupTests;
model SE2 "Reproduces test_group_se2"
  constant Real tolerance = 1.0e-9;
  constant Real tangent[3] = {1.0, 2.0, 0.3};
  Real algebraMatrix[3, 3];
  Real algebraRoundTrip[3];
  Real element[3];
  Real identityProduct[3];
  Real inverseIdentity[3];
  Real expLog[3];
  Real groupLogExp[3];
  Real matrixRepresentation[3, 3];
  Real Ad[3, 3];
  Real ad[3, 3];
equation
  algebraMatrix = LieGroups.SE2.wedge(tangent);
  algebraRoundTrip = LieGroups.SE2.vee(algebraMatrix);
  element = LieGroups.SE2.exp_map(tangent);
  identityProduct = LieGroups.SE2.product(element, zeros(3));
  inverseIdentity = LieGroups.SE2.product(element, LieGroups.SE2.inverse(element));
  expLog = LieGroups.SE2.log_map(element);
  groupLogExp = LieGroups.SE2.exp_map(LieGroups.SE2.log_map(element));
  matrixRepresentation = LieGroups.SE2.to_Matrix(element);
  Ad = LieGroups.SE2.adjoint(element);
  ad = LieGroups.SE2.small_adjoint(tangent);
  assert(Tests.Assertions.maxAbsVector(algebraRoundTrip - tangent) < tolerance,
    "se2 wedge/vee failed");
  assert(Tests.Assertions.maxAbsVector(identityProduct - element) < tolerance,
    "SE2 identity product failed");
  assert(Tests.Assertions.maxAbsVector(inverseIdentity) < tolerance,
    "SE2 inverse failed");
  assert(Tests.Assertions.maxAbsVector(expLog - tangent) < tolerance,
    "SE2 algebra exp/log failed");
  assert(Tests.Assertions.maxAbsVector(groupLogExp - element) < tolerance,
    "SE2 group exp/log failed");
  assert(Tests.Assertions.isFiniteMatrix(matrixRepresentation) and
         Tests.Assertions.isFiniteMatrix(Ad) and
         Tests.Assertions.isFiniteMatrix(ad),
    "SE2 matrix/adjoint construction failed");
end SE2;
