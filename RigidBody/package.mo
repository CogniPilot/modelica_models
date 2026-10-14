within;
package RigidBody "Reusable rigid-body dynamics"
  annotation(uses(LieGroups, LinearAlgebra),
    Documentation(info = "<html>
    <h4>State and forces</h4>
<p><a href=\"modelica://RigidBody.State\">State</a> stores world position, body
velocity, a scalar-first body-to-world quaternion and body angular velocity.
<a href=\"modelica://RigidBody.Wrench\">Wrench</a> supplies non-gravity body force
and body torque. Gravity is applied by the dynamics in world negative z.</p>
<p>Use <a href=\"modelica://RigidBody.stateDerivative\">stateDerivative</a> for an
explicit vector field or extend
<a href=\"modelica://RigidBody.RigidBody6DOF\">RigidBody6DOF</a> for a continuous
Modelica plant. Mass must be positive and inertia symmetric positive definite.</p>
<h4>Level hover example</h4>
<pre>model LevelHover
  extends RigidBody.RigidBody6DOF(mass=1.5);
equation
  F_b = {0,0,mass*g};
  M_b = zeros(3);
end LevelHover;</pre>
<p>At the default level orientation and zero initial motion, thrust balances
gravity. Add vehicle forces in a derived model rather than including gravity
in F_b a second time. Energy and power helpers support independent checks of
force, velocity and frame conventions.</p>
    </html>"));
end RigidBody;
