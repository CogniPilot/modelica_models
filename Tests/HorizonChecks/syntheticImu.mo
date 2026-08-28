within Tests.HorizonChecks;

function syntheticImu
  "One sample of a coning-rich and sculling-rich synthetic inertial stream"
  input Integer index(min = 1);
  input Real dt(unit = "s");
  output Real angularVelocityBodyFlu_rad_s[3](each unit = "rad/s");
  output Real specificForceBodyFlu_m_s2[3](each unit = "m/s2");
protected
  constant Real pi = 3.1415926535897932;
  Real t;
algorithm
  t := (index - 1) * dt;
  // A rotating angular-velocity vector at 30 Hz with a steady yaw rate: the
  // rotation axis moves, so the coning term is driven and a composition that
  // dropped it would show. Specific force runs on three incommensurate
  // frequencies so the sculling and scrolling terms are driven independently
  // of the rotation.
  angularVelocityBodyFlu_rad_s := {
    0.6 * sin(2.0 * pi * 30.0 * t),
    0.6 * cos(2.0 * pi * 30.0 * t),
    0.35};
  specificForceBodyFlu_m_s2 := {
    1.5 * sin(2.0 * pi * 17.0 * t),
    -1.1 * cos(2.0 * pi * 23.0 * t),
    9.81 + 0.8 * sin(2.0 * pi * 11.0 * t)};
end syntheticImu;
