within Vehicles;

package Ground "Sampled low-speed ground vehicle dynamics and rigid offsets"
  annotation(Documentation(info="<html>
<p><code>SprungBicycleStep</code> advances a four-wheel vehicle in ENU using
a bicycle steering approximation and a sprung chassis. Supply current
terrain heights in front-left, front-right, rear-left, rear-right order.
The caller owns the previous sample and applies the returned state at the
next step. Acceleration and braking commands are normalized to [0,1],
steering to [-1,1], and boost is either zero or one.</p>
<p>The drivetrain is a four-speed forward-only approximation. The suspension
models heave and terrain-induced body pitch and roll. Tire slip, contact
loss, and high-speed handling are outside this model's domain.</p>
<p><code>bodyOffsetPointENU</code> resolves a rigid point from right-forward-up
body coordinates into ENU.</p>
</html>"));
end Ground;
