within Tests.LieGroupTests;
model ChartContract "Contract checks every SO(3) chart must pass; instantiate with a redeclared Chart"
  replaceable package Chart = LieGroups.SO3.Charts.Exp
    constrainedby LieGroups.SO3.Charts.PartialChart;
  constant Real tolerance = 1.0e-12;
  constant Real fdTolerance = 1.0e-7 "Central difference with step 1e-5";
  constant Real fdStep = 1.0e-5;
  constant Real basis[3, 3] = identity(3) "Columns are the unit vectors e_k";
  parameter Real delta[3] = {0.3, -0.2, 0.25} "Test point; must satisfy Chart.inDomain";
  constant Real omega[3] = {0.7, -1.1, 0.4};
  constant Real q0[4] = {0.8775825618903728, 0.0, 0.479425538604203, 0.0} "Base: 1 rad about y";
  Real qZero[4];
  Real JZero[3, 3];
  Real q[4];
  Real deltaRoundTrip[3];
  Real deltaLocal[3];
  Real J[3, 3];
  Real JInv[3, 3];
  Real JFd[3, 3] "Central difference of to_Quat in the library's right-trivialized rule convention";
  Real deltaDot[3];
  Real qMinus[4];
  Real ADelta[3, 3] "d(rate(delta, omega))/d(delta) at delta = 0: orientation block of a linearization in this chart";
equation
  qZero = Chart.to_Quat(zeros(3));
  JZero = Chart.to_Quat_jacobian(zeros(3));
  q = Chart.to_Quat(delta);
  deltaRoundTrip = Chart.from_Quat(q);
  deltaLocal = Chart.local(q0, Chart.retract(q0, delta));
  J = Chart.to_Quat_jacobian(delta);
  JInv = Chart.to_Quat_jacobian_inv(delta);
  for k in 1:3 loop
    JFd[:, k] = (LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
                   LieGroups.SO3.Quat.inverse(q), Chart.to_Quat(delta + fdStep * basis[:, k])))
               - LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.product(
                   LieGroups.SO3.Quat.inverse(q), Chart.to_Quat(delta - fdStep * basis[:, k]))))
               / (2.0 * fdStep);
  end for;
  deltaDot = Chart.rate(delta, omega);
  qMinus = Chart.to_Quat(-delta);
  for k in 1:3 loop
    ADelta[:, k] = (Chart.rate(fdStep * basis[:, k], omega)
                  - Chart.rate(-fdStep * basis[:, k], omega)) / (2.0 * fdStep);
  end for;

  assert(abs(abs(qZero[1]) - 1.0) < tolerance and Tests.Assertions.maxAbsVector(qZero[2:4]) < tolerance,
    "Chart to_Quat(0) must be the identity");
  assert(Tests.Assertions.maxAbsMatrix(JZero - identity(3)) < tolerance,
    "Chart Jacobian at zero must be the identity (normalized chart)");
  assert(abs(q * q - 1.0) < tolerance, "Chart to_Quat must return a unit quaternion");
  assert(Tests.Assertions.maxAbsVector(deltaRoundTrip - delta) < 1.0e-10, "Chart from_Quat(to_Quat(delta)) round trip failed");
  assert(Tests.Assertions.maxAbsVector(deltaLocal - delta) < 1.0e-10, "Chart local(q0, retract(q0, delta)) round trip failed");
  assert(Tests.Assertions.maxAbsMatrix(J - JFd) < fdTolerance,
    "Chart to_Quat_jacobian disagrees with the central difference of to_Quat");
  assert(Tests.Assertions.maxAbsMatrix(J * JInv - identity(3)) < tolerance, "Chart Jacobian inverse failed");
  assert(Tests.Assertions.maxAbsVector(J * deltaDot - omega) < tolerance, "Chart rate must reproduce body angular velocity");
  assert(Chart.inDomain(delta), "Test point must lie in the chart domain");
  assert(not Chart.symmetric or abs(abs(qMinus * LieGroups.SO3.Quat.inverse(q)) - 1.0) < tolerance,
    "Symmetric chart must satisfy to_Quat(-delta) = to_Quat(delta)^(-1)");
  assert(Chart.symmetric or abs(abs(qMinus * LieGroups.SO3.Quat.inverse(q)) - 1.0) > 1.0e-6,
    "Non-symmetric chart unexpectedly satisfies the symmetry identity");
  assert(not Chart.symmetric or
         Tests.Assertions.maxAbsMatrix(ADelta + 0.5 * LieGroups.SO3.Quat.wedge(omega)) < fdTolerance,
    "Symmetric chart linearization must give the orientation block -[omega]x/2");
  assert(Chart.symmetric or
         Tests.Assertions.maxAbsMatrix(ADelta + 0.5 * LieGroups.SO3.Quat.wedge(omega)) > 1.0e-3,
    "Non-symmetric chart unexpectedly reproduced the symmetric-chart linearization");
end ChartContract;
