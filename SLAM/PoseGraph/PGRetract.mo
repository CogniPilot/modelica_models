within SLAM.PoseGraph;
function PGRetract
  import PGExp = SLAM.PoseGraph.PGExp;

  input Real p[:,3]; input Real R[size(p,1),3,3]; input Real nodeMask[size(p,1)]; input Real delta[size(p,1),6]; input Real scale;
  output Real nextP[size(p,1),3]; output Real nextR[size(p,1),3,3];
algorithm
  nextP := p; nextR := R;
  for node in 1:size(p,1) loop
    if node > 1 and nodeMask[node] == 1.0 then
      nextP[node,:] := p[node,:]+scale*delta[node,1:3]; nextR[node,:,:] := R[node,:,:]*PGExp(scale*delta[node,4:6]);
    end if;
  end for;
end PGRetract;
