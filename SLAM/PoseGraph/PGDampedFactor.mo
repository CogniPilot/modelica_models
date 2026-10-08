within SLAM.PoseGraph;
function PGDampedFactor
  import PGCholesky = SLAM.PoseGraph.PGCholesky;

  input Real localBlock[6,6]; input Real damping;
  output Real L[6,6]; output Real diagonal[6]; output Boolean valid;
protected Real A[6,6];
algorithm
  A := localBlock; diagonal := zeros(6);
  for k in 1:6 loop diagonal[k] := max(A[k,k],1e-6); A[k,k] := A[k,k]+damping*diagonal[k]; end for;
  (L,valid) := PGCholesky(A,1e-12);
end PGDampedFactor;
