within;
package Control "Reusable control algorithms"
  annotation(
    uses(LieGroups, LinearAlgebra, MathUtilities, Planning),
    Documentation(info="<html>
    <p>Execution-neutral feedback, guidance, and control components. Pure
    control maps are kept independent of vehicle parameterizations; named
    vehicles extend the reusable models with their physical constants and
    tuning.</p>
  <h4>Find a controller</h4>
<ul>
<li><a href=\"modelica://Control.PidController\">PidController</a>: sampled PID
with command limits and tracking anti-windup.</li>
<li><a href=\"modelica://Control.Multirotor.LogLinear\">Multirotor.LogLinear</a>:
SE_2(3) position/velocity tracking and SO(3) attitude control.</li>
<li><a href=\"modelica://Control.Multirotor.RateLoop\">Multirotor.RateLoop</a>:
body-rate feedback.</li>
<li><a href=\"modelica://Control.Multirotor.Allocation\">Multirotor.Allocation</a>:
conversion from requested wrench to rotor commands.</li>
<li><a href=\"modelica://Control.Mpc\">Mpc</a>: receding-horizon problem interfaces
and transcriptions.</li>
</ul>
<h4>Connect the control chain</h4>
<p>A planner supplies position, velocity, acceleration and heading references.
A navigation estimate supplies the measured physical state. Outer-loop commands
feed attitude/rate control and allocation, then a plant under
<a href=\"modelica://Vehicles\">Vehicles</a>. Keep reference and measurement frames
consistent; transport and actuator protocols belong at the vehicle boundary.</p>
    </html>"));
end Control;
