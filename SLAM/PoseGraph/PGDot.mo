within SLAM.PoseGraph;
function PGDot
  input Real x[:,:]; input Real y[size(x,1),size(x,2)]; output Real value;
algorithm
  value := 0.0;
  for i in 1:size(x,1) loop for j in 1:size(x,2) loop value := value+x[i,j]*y[i,j]; end for; end for;
end PGDot;
