within SLAM.PoseGraph;
model ModelicaPoseGraphEdge
  import PGEdgeDiagnostics = SLAM.PoseGraph.PGEdgeDiagnostics;

  input Real pi[3] = zeros(3); input Real pj[3] = zeros(3);
  input Real Ri[3,3] = identity(3); input Real Rj[3,3] = identity(3);
  input Real translation[3] = zeros(3); input Real measuredRotation[3,3] = identity(3);
  output Real residual[6]; output Real Ji[6,6]; output Real Jj[6,6]; output Real valid;
equation
  (residual,Ji,Jj,valid) = PGEdgeDiagnostics(pi,Ri,pj,Rj,translation,measuredRotation);
end ModelicaPoseGraphEdge;
