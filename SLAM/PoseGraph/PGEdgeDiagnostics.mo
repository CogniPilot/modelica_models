within SLAM.PoseGraph;
function PGEdgeDiagnostics
  import PGEdge = SLAM.PoseGraph.PGEdge;

  input Real pi[3]; input Real Ri[3,3]; input Real pj[3]; input Real Rj[3,3]; input Real translation[3]; input Real measuredRotation[3,3];
  output Real residual[6]; output Real Ji[6,6]; output Real Jj[6,6]; output Real valid;
protected Boolean chartValid;
algorithm
  (residual,Ji,Jj,chartValid) := PGEdge(pi,Ri,pj,Rj,translation,measuredRotation); valid := if chartValid then 1.0 else 0.0;
end PGEdgeDiagnostics;
