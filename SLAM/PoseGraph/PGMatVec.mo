within SLAM.PoseGraph;
function PGMatVec
  input Real A[3,3]; input Real x[3]; output Real y[3];
algorithm
  y[1] := A[1,1]*x[1]+A[1,2]*x[2]+A[1,3]*x[3];
  y[2] := A[2,1]*x[1]+A[2,2]*x[2]+A[2,3]*x[3];
  y[3] := A[3,1]*x[1]+A[3,2]*x[2]+A[3,3]*x[3];
end PGMatVec;
