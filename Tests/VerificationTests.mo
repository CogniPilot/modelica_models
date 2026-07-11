within Tests;
model VerificationTests
  "Proof-facing vector-field, storage, chart-domain, and contraction identities"
  function run
    output Boolean passed;
  protected
    constant Real pi = 2.0 * asin(1.0);
    RigidBody.State state;
    RigidBody.StateDerivative stateRate;
    RigidBody.Wrench wrench;
    RigidBody.Parameters parameters;
    Real R[3, 3];
    Real negativeQuaternionR[3, 3];
    Real cutRotation[3, 3];
    Real contraction[2, 2];
    Real tangent[6];
    LieGroups.SE3.WithQuaternion.Element reference;
    LieGroups.SE3.WithQuaternion.Element actual;
  algorithm
    state := RigidBody.State(
      worldPosition={1.0, -2.0, 3.0},
      bodyVelocity={0.4, -0.1, 0.2},
      attitude=LieGroups.SO3.Quat.exp_map({0.2, -0.1, 0.3}),
      bodyAngularVelocity={0.5, -0.2, 0.1});
    wrench := RigidBody.Wrench(
      bodyForce={2.0, -1.0, 0.5},
      bodyTorque={0.1, 0.3, -0.2});
    parameters := RigidBody.Parameters(
      mass=2.5,
      gravity=9.81,
      inertia=[0.4, 0.02, 0.0; 0.02, 0.6, -0.01; 0.0, -0.01, 0.8],
      quaternionNormGain=0.0);
    stateRate := RigidBody.stateDerivative(state, wrench, parameters);
    assert(RigidBody.validParameters(parameters),
      "Valid rigid-body parameters were rejected");
    assert(abs(RigidBody.powerBalanceResidual(
        state, stateRate, wrench, parameters)) < 1.0e-10,
      "Rigid-body vector field violated the mechanical power balance");
    assert(abs(RigidBody.portPower(wrench, RigidBody.bodyTwist(state))
        - RigidBody.wrenchPower(state, wrench)) < 1.0e-12,
      "Rigid-body wrench/twist power port was inconsistent");
    assert(RigidBody.mechanicalEnergy(state, parameters) ==
        RigidBody.kineticEnergy(state, parameters)
          + RigidBody.potentialEnergy(state, parameters),
      "Mechanical storage decomposition was inconsistent");

    R := LieGroups.SO3.Quat.to_DCM(state.attitude);
    negativeQuaternionR := LieGroups.SO3.Quat.to_DCM(-state.attitude);
    cutRotation := LieGroups.SO3.Dcm.exp_map({pi, 0.0, 0.0});
    assert(abs(LieGroups.Analysis.attitudePotential(R)
        - LieGroups.Analysis.attitudePotential(negativeQuaternionR)) < 1.0e-12,
      "SO(3) potential depended on quaternion sign");
    assert(LieGroups.Analysis.inPrincipalLogDomain(R, 0.1) and
        not LieGroups.Analysis.inPrincipalLogDomain(cutRotation, 1.0e-4),
      "Principal-log chart-domain guard failed");
    assert(LieGroups.SO3.Euler.inNonsingularChart(
          {0.2, 0.7, -0.1},
          {LieGroups.SO3.Euler.Axis.y, LieGroups.SO3.Euler.Axis.z,
            LieGroups.SO3.Euler.Axis.y}, 0.1) and
        LieGroups.SO3.Euler.inNonsingularChart(
          {0.2, 0.3, -0.1},
          {LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y,
            LieGroups.SO3.Euler.Axis.z}, 0.1) and
        not LieGroups.SO3.Euler.inNonsingularChart(
          {0.2, pi / 2.0, -0.1},
          {LieGroups.SO3.Euler.Axis.x, LieGroups.SO3.Euler.Axis.y,
            LieGroups.SO3.Euler.Axis.z}, 0.1),
      "Euler chart-domain guards failed for proper and Tait-Bryan sequences");
    assert(LieGroups.SO3.Mrp.inPrincipalChart({0.1, -0.2, 0.3}, 0.1) and
        not LieGroups.SO3.Mrp.inPrincipalChart({1.0, 0.0, 0.0}, 0.1),
      "MRP shadow-switch chart guard failed");
    assert(LieGroups.Analysis.logQuadraticPotential(R) > 0.0,
      "Log-coordinate Lyapunov candidate was not positive away from identity");

    contraction := LinearAlgebra.contractionResidual(
      [-2.0, 0.0; 0.0, -3.0], identity(2), zeros(2, 2), 1.0);
    assert(Tests.Assertions.maxAbsMatrix(
        contraction - [-2.0, 0.0; 0.0, -4.0]) < 1.0e-12,
      "Dimension-generic contraction residual was incorrect");
    assert(abs(LinearAlgebra.quadraticFormDerivative(
        {1.0, 2.0}, {-1.0, -2.0}, identity(2)) + 10.0) < 1.0e-12,
      "Quadratic-form derivative was incorrect");

    tangent := {0.3, -0.2, 0.1, 0.05, -0.03, 0.02};
    reference := LieGroups.SE3.WithQuaternion.identity();
    actual := LieGroups.SE3.WithQuaternion.exp_map(tangent);
    assert(Tests.Assertions.maxAbsVector(
        LieGroups.SE3.WithQuaternion.leftInvariantError(reference, actual)
          - tangent) < 1.0e-9,
      "Left-invariant SE(3) error coordinate was inconsistent with exp/log");
    passed := true;
  end run;

  parameter Boolean passed = run();
equation
  assert(passed, "Verification primitive assertions did not complete");
end VerificationTests;
