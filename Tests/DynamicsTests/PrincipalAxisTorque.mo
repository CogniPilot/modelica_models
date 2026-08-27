within Tests.DynamicsTests;
model PrincipalAxisTorque "Principal-axis angular acceleration has an analytic solution"
  constant Real appliedTorque = 0.6;
  extends RigidBody.RigidBody6DOF(
    mass=1.0,
    g=0.0,
    ixx=0.3,
    iyy=0.5,
    izz=0.7,
    q_start={1.0, 0.0, 0.0, 0.0},
    omega_start={0.0, 0.0, 0.0});
equation
  F_b = zeros(3);
  M_b = {appliedTorque, 0.0, 0.0};
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
  assert(abs(omega[1] - appliedTorque / ixx * time) < 2.0e-5,
    "Principal-axis angular velocity did not match the analytic solution");
  assert(abs(omega[2]) < 1.0e-8 and abs(omega[3]) < 1.0e-8,
    "Principal-axis torque excited another angular axis");
  assert(abs(q * q - 1.0) < 5.0e-6,
    "Principal-axis rotation did not preserve quaternion norm; absolute error = "
      + String(abs(q * q - 1.0), significantDigits=16));
end PrincipalAxisTorque;
