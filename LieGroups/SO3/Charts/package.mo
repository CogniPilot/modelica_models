within LieGroups.SO3;
package Charts "Swappable local coordinate charts on SO(3) about a base orientation"
  annotation(Documentation(info="<html>
    <p>A chart gives three local coordinates <code>delta</code> to a neighbourhood of a base
    orientation <code>q0</code>:</p>
    <pre>  q = q0 * to_Quat(delta)          (retract)
  delta = from_Quat(q0^(-1) * q)   (local)</pre>
    <p>This is right-composed, matching the library's right trivialization. Every chart is normalized so that
    <code>to_Quat(0)</code> is the identity and its derivative at zero is the identity, so all charts agree to
    first order at the base.</p>
    <p><code>to_Quat_jacobian(delta)</code> is the rule for <code>to_Quat</code> in the library convention:
    <code>log_map(to_Quat(delta)^(-1) * to_Quat(delta + d)) = J*d + O(|d|^2)</code>. It maps chart rates to body
    angular velocity, so a model with body-frame <code>omega</code> writes
    <code>der(delta) = Chart.rate(delta, omega)</code>.</p>
    <p><b>Swapping.</b> Write a model against
    <code>replaceable package Chart = ... constrainedby LieGroups.SO3.Charts.PartialChart</code> and
    redeclare it. A tool's linearization of that model is then taken in the chosen chart. See
    <a href=\"modelica://LieGroups.SO3.Charts.Examples.RigidBodyInChart\">Examples.RigidBodyInChart</a>.
    For long runs, <a href=\"modelica://LieGroups.SO3.Charts.Examples.RigidBodyRecentered\">Examples.RigidBodyRecentered</a>
    keeps the chart small by recentering its base at events. For fixed-step targets,
    <a href=\"modelica://LieGroups.SO3.Charts.Examples.RigidBodyLieEuler\">Examples.RigidBodyLieEuler</a> steps the
    orientation directly on the group.</p>
    <p><b>Which chart matters when.</b> The symmetric charts <a href=\"modelica://LieGroups.SO3.Charts.Exp\">Exp</a>,
    <a href=\"modelica://LieGroups.SO3.Charts.Cayley\">Cayley</a> and <a href=\"modelica://LieGroups.SO3.Charts.Mrp\">Mrp</a>
    satisfy <code>to_Quat(-delta) = to_Quat(delta)^(-1)</code>, so their transition maps differ from each other
    only at third order. A linearization at <code>delta = 0</code> therefore gives identical A and B in any of them, even off
    equilibrium. <a href=\"modelica://LieGroups.SO3.Charts.EulerB321\">EulerB321</a> is not symmetric, and gives a
    different A whenever the linearization point is not an equilibrium. The choice among symmetric charts matters
    away from the base: validity range, distance to singularity, and cost.</p>
  </html>"));
end Charts;
