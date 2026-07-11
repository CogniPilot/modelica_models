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
  when terminal() then
    assert(abs(p[1] - 1.0) < 1.0e-6 and abs(p[2] + 2.0) < 1.0e-6,
      "Free fall produced horizontal translation");
    assert(abs(p[3] - (10.0 - 0.5 * g * time * time)) < 2.0e-4,
      "Free-fall position did not match the analytic solution");
    assert(abs(v_w[3] + g * time) < 2.0e-4,
      "Free-fall velocity did not match the analytic solution");
    assert(abs(q * q - 1.0) < 1.0e-8,
      "Free fall did not preserve quaternion norm");
  end when;
end FreeFall;
