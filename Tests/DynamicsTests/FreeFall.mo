within Tests.DynamicsTests;
model FreeFall "Unforced translation follows the analytic free-fall solution"
  extends RigidBody.RigidBody6DOF(
    mass=2.0,
    g=9.81,
    p_start={1.0, -2.0, 10.0},
    v_b_start={0.0, 0.0, 0.0},
    q_start={1.0, 0.0, 0.0, 0.0},
    omega_start={0.0, 0.0, 0.0});
equation
  F_b = zeros(3);
  M_b = zeros(3);
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
  assert(abs(p[1] - 1.0) < 1.0e-6 and abs(p[2] + 2.0) < 1.0e-6,
    "Free fall produced horizontal translation");
  assert(abs(p[3] - (10.0 - 0.5 * g * time * time)) < 2.0e-4,
    "Free-fall position did not match the analytic solution");
  assert(abs(v_w[3] + g * time) < 2.0e-4,
    "Free-fall velocity did not match the analytic solution");
  assert(abs(q * q - 1.0) < 1.0e-8,
    "Free fall did not preserve quaternion norm");
end FreeFall;
