within Tests.LieGroupTests;
model RnAndDirectProduct "Reproduces test_group_rn and test_direct_product"
  constant Real tolerance = 1.0e-10;
  constant Real x[3] = {1.0, 2.0, 3.0};
  constant Real y[3] = {4.0, 5.0, 6.0};
  constant Real directElement[9] = {0.1, 0.2, 0.3, 4, 5, 6, 7, 8, 9};
  Real product[3];
  Real identityProduct[3];
  Real inverseProduct[3];
  Real homogeneous[4, 4];
  Real algebraMatrix[4, 4];
  Real vee[3];
  Real Ad[3, 3];
  Real ad[3, 3];
  Real expLog[3];
  Real directProduct[9];
  Real directIdentity[9];
  Real directInverse[9];
  Real directExpLog[9];
  Real directAdjoint[9, 9];
equation
  product = LieGroups.Rn.product(x, y);
  identityProduct = LieGroups.Rn.product(x, zeros(3));
  inverseProduct = LieGroups.Rn.product(x, LieGroups.Rn.inverse(x));
  homogeneous = LieGroups.Rn.to_Matrix(x);
  algebraMatrix = LieGroups.Rn.wedge(x);
  vee = LieGroups.Rn.vee(algebraMatrix);
  Ad = LieGroups.Rn.adjoint(x);
  ad = LieGroups.Rn.small_adjoint(x);
  expLog = LieGroups.Rn.exp_map(LieGroups.Rn.log_map(x));

  directProduct = LieGroups.DirectProduct.SE2R3R3.product(
    directElement, zeros(9));
  directIdentity = LieGroups.DirectProduct.SE2R3R3.product(
    directElement,
    LieGroups.DirectProduct.SE2R3R3.inverse(directElement));
  directInverse = LieGroups.DirectProduct.SE2R3R3.inverse(directElement);
  directExpLog = LieGroups.DirectProduct.SE2R3R3.exp_map(
    LieGroups.DirectProduct.SE2R3R3.log_map(directElement));
  directAdjoint = LieGroups.DirectProduct.SE2R3R3.adjoint(directElement);

  assert(Tests.Assertions.maxAbsVector(product - (x + y)) < tolerance,
    "R3 product test failed");
  assert(Tests.Assertions.maxAbsVector(identityProduct - x) < tolerance,
    "R3 identity test failed");
  assert(Tests.Assertions.maxAbsVector(inverseProduct) < tolerance,
    "R3 inverse test failed");
  assert(Tests.Assertions.maxAbsVector(vee - x) < tolerance,
    "r3 wedge/vee test failed");
  assert(Tests.Assertions.maxAbsMatrix(Ad - identity(3)) < tolerance and
         Tests.Assertions.maxAbsMatrix(ad) < tolerance,
    "R3 adjoint tests failed");
  assert(Tests.Assertions.maxAbsVector(expLog - x) < tolerance,
    "R3 exp/log test failed");
  assert(Tests.Assertions.maxAbsVector(directProduct - directElement) < tolerance,
    "direct-product product test failed");
  assert(Tests.Assertions.maxAbsVector(directIdentity) < tolerance,
    "direct-product inverse test failed");
  assert(Tests.Assertions.maxAbsVector(directExpLog - directElement) < tolerance,
    "direct-product exp/log test failed");
  assert(Tests.Assertions.isFiniteMatrix(homogeneous) and
         Tests.Assertions.isFiniteMatrix(directAdjoint),
    "R3/direct-product matrix construction failed");
end RnAndDirectProduct;
