within ;
package MathUtilities "General scalar and array math utilities"
  annotation(Documentation(info = "<html>
    <h4>Signal shaping and limits</h4>
<p>Small scalar and array functions for control and estimation. Use
<a href=\"modelica://MathUtilities.clip\">clip</a> for component bounds,
<a href=\"modelica://MathUtilities.limitNorm\">limitNorm</a> for a vector magnitude
bound, and <a href=\"modelica://MathUtilities.wrapAngle\">wrapAngle</a> for periodic
angles. Component clipping and norm limiting have different geometry.</p>
<p><a href=\"modelica://MathUtilities.lowPass\">lowPass</a>,
<a href=\"modelica://MathUtilities.lowPass3\">lowPass3</a> and
<a href=\"modelica://MathUtilities.rateLimit\">rateLimit</a> are explicit update
maps: the caller supplies prior state and timing. They do not create a hidden
sampling clock. Match argument units and update intervals to the calling model.</p>
    </html>"));
end MathUtilities;
