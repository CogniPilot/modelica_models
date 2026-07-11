within Tests;
model LinearAlgebraTests "Generic matrix sizes, success paths, and failure paths"
  constant Real tolerance = 1.0e-10;
  constant Real A1[1, 1] = [4.0];
  constant Real B1[1, 2] = [8.0, -4.0];
  constant Real A3[3, 3] = [
    4.0, 1.0, 1.0;
    1.0, 3.0, 0.5;
    1.0, 0.5, 2.0];
  constant Real B3[3, 2] = [
    1.0, 2.0;
    -1.0, 0.5;
    3.0, -2.0];
  constant Real A5[5, 5] = [
    5.0, 1.0, 0.0, 0.0, 0.0;
    1.0, 5.0, 1.0, 0.0, 0.0;
    0.0, 1.0, 5.0, 1.0, 0.0;
    0.0, 0.0, 1.0, 5.0, 1.0;
    0.0, 0.0, 0.0, 1.0, 5.0];
  constant Real B5[5, 3] = [
    1.0, 0.0, -1.0;
    2.0, 1.0, 0.0;
    3.0, 0.0, 1.0;
    4.0, -1.0, 0.0;
    5.0, 2.0, 1.0];
  constant Real indefinite[2, 2] = [1.0, 2.0; 2.0, 1.0];
  constant Real asymmetric[3, 3] = [
    1.0, 2.0, 3.0;
    4.0, 5.0, 6.0;
    7.0, 8.0, 9.0];
  Real residual1;
  Real residual3;
  Real residual5;
  Real acceptedIndefinite;
  Real symmetric3[3, 3];
  Real joseph[3, 3];
equation
  residual1 = Tests.Assertions.spdResidual(A1, B1);
  residual3 = Tests.Assertions.spdResidual(A3, B3);
  residual5 = Tests.Assertions.spdResidual(A5, B5);
  acceptedIndefinite = Tests.Assertions.spdAccepted(indefinite);
  symmetric3 = LinearAlgebra.symmetrize(asymmetric);
  joseph = LinearAlgebra.josephUpdate(
    identity(3),
    identity(3),
    zeros(3, 2),
    identity(2));

  assert(acceptedIndefinite < 0.5, "solveSPD accepted an indefinite matrix");
  assert(residual1 < tolerance,
    "solveSPD 1x1 residual exceeded tolerance");
  assert(residual3 < tolerance,
    "solveSPD 3x3 residual exceeded tolerance");
  assert(residual5 < tolerance,
    "solveSPD 5x5 residual exceeded tolerance");
  assert(Tests.Assertions.maxAbsMatrix(symmetric3 - transpose(symmetric3)) < tolerance,
    "symmetrize did not return a symmetric matrix");
  assert(abs(symmetric3[1, 2] - 3.0) < tolerance,
    "symmetrize did not average opposing entries");
  assert(Tests.Assertions.maxAbsMatrix(joseph - identity(3)) < tolerance,
    "generic Joseph update returned an unexpected result");
end LinearAlgebraTests;
