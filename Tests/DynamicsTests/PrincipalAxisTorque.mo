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
  when terminal() then
    assert(abs(omega[1] - appliedTorque / ixx * time) < 2.0e-5,
      "Principal-axis angular velocity did not match the analytic solution");
    assert(abs(omega[2]) < 1.0e-8 and abs(omega[3]) < 1.0e-8,
      "Principal-axis torque excited another angular axis");
    assert(abs(q * q - 1.0) < 2.0e-6,
      "Principal-axis rotation did not preserve quaternion norm");
  end when;
end PrincipalAxisTorque;
