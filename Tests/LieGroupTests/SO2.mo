within Tests.LieGroupTests;
model SO2 "Reproduces test_group_so2"
  constant Real tolerance = 1.0e-10;
  constant Real first = 1.0;
  constant Real second = 2.0;
  Real algebraMatrix[2, 2];
  Real groupMatrix[2, 2];
  Real groupProduct;
  Real inverseIdentity;
  Real expLog;
  Real Ad[1, 1];
  Real ad[1, 1];
equation
  algebraMatrix = LieGroups.SO2.wedge(first);
  groupMatrix = LieGroups.SO2.to_Matrix(first);
  groupProduct = LieGroups.SO2.product(first, 0.0);
  inverseIdentity = LieGroups.SO2.product(first, LieGroups.SO2.inverse(first));
  expLog = LieGroups.SO2.exp_map(LieGroups.SO2.log_map(first));
  Ad = LieGroups.SO2.adjoint(first);
  ad = LieGroups.SO2.small_adjoint(first);
  assert(abs(LieGroups.SO2.vee(algebraMatrix) - first) < tolerance,
    "so2 wedge/vee test failed");
  assert(abs((first + second) - 3.0) < tolerance and
         abs(3.0 * first - 3.0) < tolerance,
    "so2 algebra addition/scalar multiplication failed");
  assert(abs(groupProduct - first) < tolerance,
    "SO2 identity product failed");
  assert(abs(inverseIdentity) < tolerance,
    "SO2 inverse failed");
  assert(abs(expLog - first) < tolerance,
    "SO2 exp/log failed");
  assert(Tests.Assertions.maxAbsMatrix(
      transpose(groupMatrix) * groupMatrix - identity(2)) < tolerance,
    "SO2 matrix conversion failed");
  assert(abs(Ad[1, 1] - 1.0) < tolerance and abs(ad[1, 1]) < tolerance,
    "SO2 adjoint tests failed");
end SO2;
