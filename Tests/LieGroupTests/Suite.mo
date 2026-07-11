within Tests.LieGroupTests;
model Suite "Complete Lie-group test suite"
  Tests.LieGroupTests.RnAndDirectProduct rnAndDirectProduct;
  Tests.LieGroupTests.SO2 so2;
  Tests.LieGroupTests.SE2 se2;
  Tests.LieGroupTests.SO3 so3;
  Tests.LieGroupTests.SE3 se3;
  Tests.LieGroupTests.SE23 se23;
  Tests.LieGroupTests.NumericalEdgeCases numericalEdgeCases;
  Tests.LieGroupTests.ModularRepresentations modularRepresentations;
end Suite;
