within SLAM;

package PoseGraph
  "Pose-graph optimization, gauge handling and estimator commits"
  annotation(Documentation(info = "<html>
    <h4>Measured relative constraints</h4>
<p><a href=\"modelica://SLAM.PoseGraph.ModelicaPoseGraph\">ModelicaPoseGraph</a>
optimizes node poses from measured translation/rotation edges and 6 by 6
information matrices. Masks select active nodes and edges; fixed capacities
and iteration limits bound storage and requested work.</p>
<p>A relative graph has gauge freedom. A chosen anchor fixes coordinates rather
than supplying an extra physical observation. Keep gauge treatment separate
from uncertainty claims; see
<a href=\"modelica://SLAM.PoseGraph.GraphGaugeUncertainty\">GraphGaugeUncertainty</a>.</p>
<p>Inspect status and cost before accepting a candidate. Publishing a graph
correction into an estimator also requires consistent landmark anchors and
reference uncertainty; the
<a href=\"modelica://SLAM.PoseGraph.RGBDGraphEstimatorCommit\">RGBDGraphEstimatorCommit</a>
boundary makes that handoff explicit. Optimizer convergence alone does not
qualify an estimator reset.</p>
    </html>"));
end PoseGraph;
