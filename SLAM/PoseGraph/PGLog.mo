within SLAM.PoseGraph;
function PGLog
  input Real R[3,3]; output Real angle[3]; output Boolean valid;
protected Real cosine; Real theta; Real square; Real scale;
algorithm
  cosine := min(1.0,max(-1.0,(R[1,1]+R[2,2]+R[3,3]-1.0)/2.0));
  theta := acos(cosine); square := theta*theta;
  // The chart is deliberately refused near its nonunique pi branch.
  valid := theta >= 0.0 and theta < 3.140592653589793;
  scale := if square < 1e-8 then 0.5+square/12.0+7.0*square*square/720.0
    else theta/(2.0*max(sin(theta),1e-12));
  angle := if valid then scale*{R[3,2]-R[2,3],R[1,3]-R[3,1],R[2,1]-R[1,2]} else zeros(3);
end PGLog;
