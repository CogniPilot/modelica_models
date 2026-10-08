within SLAM.PoseGraph;
function PGPrecondition
  import PGSolveBlock = SLAM.PoseGraph.PGSolveBlock;

  input Real rhs[:,6]; input Real factors[size(rhs,1),6,6]; input Real nodeMask[size(rhs,1)];
  output Real z[size(rhs,1),6];
algorithm
  z := zeros(size(rhs,1),6);
  for node in 1:size(rhs,1) loop
    if node > 1 and nodeMask[node] == 1.0 then z[node,:] := PGSolveBlock(factors[node,:,:],rhs[node,:]); end if;
  end for;
end PGPrecondition;
