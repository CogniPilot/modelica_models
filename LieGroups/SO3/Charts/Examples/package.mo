within LieGroups.SO3.Charts;
package Examples "Models written against a replaceable chart"
  function bodyAngularAcceleration "Euler's equations about principal axes: I*der(omega) = torque - omega x (I*omega)"
    input Real inertia[3](each unit = "kg.m2") "Principal moments of inertia, body frame";
    input Real omega[3](each unit = "rad/s") "Angular velocity, body frame";
    input Real torque[3](each unit = "N.m") "Applied torque, body frame";
    output Real omegaDot[3](each unit = "rad/s2");
  algorithm
    for i in 1:3 loop
      omegaDot[i] := (torque[i]
        - (omega[mod(i, 3) + 1] * inertia[mod(i + 1, 3) + 1] * omega[mod(i + 1, 3) + 1]
           - omega[mod(i + 1, 3) + 1] * inertia[mod(i, 3) + 1] * omega[mod(i, 3) + 1])) / inertia[i];
    end for;
  end bodyAngularAcceleration;

  model RigidBodyInChart
    "Rigid body with orientation in a replaceable chart about a fixed base; linearize it to get A, B in that chart"
    replaceable package Chart = LieGroups.SO3.Charts.Exp
      constrainedby LieGroups.SO3.Charts.PartialChart "Orientation chart";
    parameter Real q0[4] = {1, 0, 0, 0} "Base orientation {w,x,y,z}, body to world";
    parameter Real inertia[3](each unit = "kg.m2") = {1.0, 1.5, 2.0} "Principal moments of inertia, body frame";
    input Real torque[3](each unit = "N.m") = zeros(3) "Applied torque, body frame";
    Real delta[3](each start = 0.0, each fixed = true) "Orientation in Chart coordinates about q0";
    Real omega[3](each unit = "rad/s", start = {0.3, -0.2, 2.0}, each fixed = true) "Angular velocity, body frame";
    output Real q[4] "Orientation {w,x,y,z}, body to world";
  equation
    der(delta) = Chart.rate(delta, omega);
    der(omega) = bodyAngularAcceleration(inertia, omega, torque);
    q = Chart.retract(q0, delta);
    annotation(Documentation(info="<html>
      <p>Euler's equations in the body frame, with orientation carried in chart coordinates:
      <code>der(delta) = Chart.rate(delta, omega)</code> and <code>q = Chart.retract(q0, delta)</code>.
      Swap the chart with, for example,
      <code>RigidBodyInChart body(redeclare package Chart = LieGroups.SO3.Charts.Cayley)</code>.</p>
      <p>Linearized at <code>delta = 0</code>, the orientation block of A is <code>-[omega]x/2</code> and its coupling
      to <code>omega</code> is the identity for every symmetric chart. EulerB321 differs when <code>omega</code> is
      not zero. The chart stays valid while <code>Chart.inDomain(delta)</code> holds; this example does not recenter.</p>
    </html>"));
  end RigidBodyInChart;

  model RigidBodyRecentered
    "Rigid body in a replaceable chart whose base is recentered by an event, so the chart stays small for all time"
    replaceable package Chart = LieGroups.SO3.Charts.Exp
      constrainedby LieGroups.SO3.Charts.PartialChart "Orientation chart";
    parameter Real q_start[4] = {1, 0, 0, 0} "Initial orientation {w,x,y,z}, body to world";
    parameter Real inertia[3](each unit = "kg.m2") = {1.0, 1.5, 2.0} "Principal moments of inertia, body frame";
    parameter Real recenterRadius(unit = "rad") = 0.5 "Recenter when |delta| exceeds this; keep it well inside Chart's domain";
    input Real torque[3](each unit = "N.m") = zeros(3) "Applied torque, body frame";
    Real delta[3](each start = 0.0, each fixed = true) "Orientation in Chart coordinates about qBase";
    Real omega[3](each unit = "rad/s", start = {0.3, -0.2, 2.0}, each fixed = true) "Angular velocity, body frame";
    discrete Real qBase[4](start = q_start, each fixed = true) "Chart base, held between recentering events";
    discrete Integer recenterCount(start = 0, fixed = true);
    output Real q[4] "Orientation {w,x,y,z}, body to world";
  equation
    der(delta) = Chart.rate(delta, omega);
    der(omega) = bodyAngularAcceleration(inertia, omega, torque);
    q = Chart.retract(qBase, delta);
    when delta * delta > recenterRadius^2 then
      qBase = LieGroups.SO3.Quat.normalize(Chart.retract(pre(qBase), pre(delta)));
      reinit(delta, zeros(3));
      recenterCount = pre(recenterCount) + 1;
    end when;
    assert(Chart.inDomain(delta), "RigidBodyRecentered: delta left the chart domain; lower recenterRadius");
    annotation(Documentation(info="<html>
      <p>The orientation is integrated as an ordinary ODE in chart coordinates, which any variable-step solver can
      handle. When <code>|delta|</code> exceeds <code>recenterRadius</code>, an event folds the offset into the base,
      <code>qBase := qBase * to_Quat(delta)</code>, and resets <code>delta</code> to zero. The physical orientation
      <code>q</code> is unchanged by the event. This is integration in local coordinates with lazy, event-driven
      recentering; each recentering restarts the solver's history.</p>
    </html>"));
  end RigidBodyRecentered;

  model RigidBodyLieEuler
    "Rigid body advanced by a fixed-step semi-implicit Lie-Euler step on SO(3), written as discrete equations"
    parameter Real h(unit = "s") = 1.0e-3 "Step size";
    parameter Real q_start[4] = {1, 0, 0, 0} "Initial orientation {w,x,y,z}, body to world";
    parameter Real omega_start[3](each unit = "rad/s") = {0.3, -0.2, 2.0} "Initial angular velocity, body frame";
    parameter Real inertia[3](each unit = "kg.m2") = {1.0, 1.5, 2.0} "Principal moments of inertia, body frame";
    input Real torque[3](each unit = "N.m") = zeros(3) "Applied torque, body frame, sampled at each step";
    discrete Real q[4](start = q_start, each fixed = true) "Orientation {w,x,y,z}, body to world";
    discrete Real omega[3](each unit = "rad/s", start = omega_start, each fixed = true) "Angular velocity, body frame";
  equation
    // The first step fires at t = h, so the state at t = k*h has taken exactly k steps.
    when sample(h, h) then
      omega = pre(omega) + h * bodyAngularAcceleration(inertia, pre(omega), torque);
      q = LieGroups.SO3.Quat.normalize(LieGroups.SO3.Quat.product(pre(q), LieGroups.SO3.Quat.exp_map(h * omega)));
    end when;
    annotation(Documentation(info="<html>
      <p>Each step updates the angular velocity first and then the orientation on the group,
      <code>q := q * exp_map(h * omega)</code>, so <code>q</code> stays a unit quaternion to roundoff with no
      constraint. The method is first order and its step is fixed by the model rather than chosen by the solver.
      Use it for fixed-step targets; use RigidBodyRecentered or RigidBodyInChart with a variable-step solver.</p>
    </html>"));
  end RigidBodyLieEuler;
end Examples;
