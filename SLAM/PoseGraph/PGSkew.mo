within SLAM.PoseGraph;
function PGSkew
  input Real x[3]; output Real S[3,3];
algorithm
  S := zeros(3,3);
  S[1,2] := -x[3]; S[1,3] := x[2]; S[2,1] := x[3];
  S[2,3] := -x[1]; S[3,1] := -x[2]; S[3,2] := x[1];
end PGSkew;
