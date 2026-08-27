within Tests.DynamicsTests;
model TorqueFreeConservation "Torque-free motion conserves energy and momentum norm"
  constant Real initialOmega[3] = {0.4, -0.3, 0.2};
  constant Real initialEnergy = 0.5 * (
    0.2 * initialOmega[1]^2
    + 0.3 * initialOmega[2]^2
    + 0.5 * initialOmega[3]^2);
  constant Real initialMomentumSquared =
    (0.2 * initialOmega[1])^2
    + (0.3 * initialOmega[2])^2
    + (0.5 * initialOmega[3])^2;
  extends RigidBody.RigidBody6DOF(
    mass=1.0,
    g=0.0,
    ixx=0.2,
    iyy=0.3,
    izz=0.5,
    q_start={1.0, 0.0, 0.0, 0.0},
    omega_start=initialOmega);
  Real rotationalEnergy;
  Real momentumSquared;
equation
  F_b = zeros(3);
  M_b = zeros(3);
  rotationalEnergy = 0.5 * omega * (J * omega);
  momentumSquared = (J * omega) * (J * omega);
  // Stated as continuous assertions, not inside `when terminal()`. An assert
  // in ANY when-clause is silently ineffective under OpenModelica: the
  // runtime logs the violation, reports "Found event, previous asserts are
  // ignored", and still finishes successfully with a result file, which is
  // the only thing Tests/run.mos inspects. Verified directly: a model whose
  // sole statement is `when terminal() then assert(false, "..."); end when;`
  // exits 0 with a non-empty result file, while the same assert in an
  // equation section exits non-zero. Every assertion in this suite was
  // therefore unable to fail. Continuously is also when these properties are
  // claimed to hold, so nothing is weakened by stating them that way.
  assert(abs(rotationalEnergy - initialEnergy) < 2.0e-6,
    "Torque-free rotational energy was not conserved");
  assert(abs(momentumSquared - initialMomentumSquared) < 2.0e-6,
    "Torque-free angular momentum norm was not conserved");
  assert(abs(q * q - 1.0) < 2.0e-6,
    "Torque-free rotation did not preserve quaternion norm");
end TorqueFreeConservation;
