within LieGroups.SO3;
package Quat "SO(3) quaternion parameterization — q = {w, x, y, z}"
  annotation(Documentation(info = "<html>
    <h4>Rotation convention</h4>
<p>A unit quaternion is {w,x,y,z}, with Hamilton multiplication. A body-to-world
quaternion rotates body vectors into world coordinates with
<a href=\"modelica://LieGroups.SO3.Quat.rotate\">rotate</a>. Keep quaternion
sign ambiguity separate from the represented rotation.</p>
<h4>Example</h4>
<p>For <code>Real q[4]</code> and <code>Real v[3]</code>, this algorithm fragment
rotates the x axis by 90 degrees about positive z:</p>
<pre>q := LieGroups.SO3.Quat.exp_map({0,0,1.5707963267948966});
v := LieGroups.SO3.Quat.rotate(q, {1,0,0});</pre>
<p>The result is approximately {0,1,0}.
<a href=\"modelica://LieGroups.SO3.Quat.exp_map\">exp_map</a> accepts a rotation
vector in radians; <a href=\"modelica://LieGroups.SO3.Quat.log_map\">log_map</a>
returns the local rotation vector. Group operations expect valid rotations;
use <a href=\"modelica://LieGroups.SO3.Quat.normalize\">normalize</a> where a
caller must explicitly restore quaternion norm.</p>
    </html>"));
end Quat;
