within Tests;
model MslRotationalSmoke "Verify the pinned MSL Rotational library executes"
  Modelica.Mechanics.Rotational.Sources.ConstantTorque torque(tau_constant=1);
  Modelica.Mechanics.Rotational.Components.Inertia inertia(
    J=1, phi(start=0, fixed=true), w(start=0, fixed=true));
equation
  connect(torque.flange, inertia.flange_a);
  assert(abs(inertia.w - (torque.tau_constant/inertia.J)*time) < 1e-6,
    "MSL Rotational smoke acceleration");
  annotation(uses(Modelica(version="4.1.0")));
end MslRotationalSmoke;
