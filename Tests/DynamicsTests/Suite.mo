within Tests.DynamicsTests;
model Suite "All rigid-body dynamic tests"
  Tests.DynamicsTests.FreeFall freeFall;
  Tests.DynamicsTests.PrincipalAxisTorque principalAxisTorque;
  Tests.DynamicsTests.TorqueFreeConservation torqueFreeConservation;
end Suite;
