within SLAM.PoseGraph;
function PGGraphCost
  import PGEdge = SLAM.PoseGraph.PGEdge;

  input Real p[:,3]; input Real R[size(p,1),3,3]; input Real edgeMask[:];
  input Integer source[size(edgeMask,1)]; input Integer target[size(edgeMask,1)];
  input Real translation[size(edgeMask,1),3]; input Real measuredRotation[size(edgeMask,1),3,3];
  input Real information[size(edgeMask,1),6,6];
  output Real cost; output Boolean valid;
protected Real residual[6]; Real Ji[6,6]; Real Jj[6,6]; Boolean edgeValid;
algorithm
  cost := 0.0; valid := true; edgeValid := false; residual := zeros(6); Ji := zeros(6,6); Jj := zeros(6,6);
  for edge in 1:size(edgeMask,1) loop
    if edgeMask[edge] == 1.0 then
      (residual,Ji,Jj,edgeValid) := PGEdge(p[source[edge],:],R[source[edge],:,:],
        p[target[edge],:],R[target[edge],:,:],translation[edge,:],measuredRotation[edge,:,:]);
      valid := valid and edgeValid; cost := cost+0.5*(residual*(information[edge,:,:]*residual));
    end if;
  end for;
  valid := valid and cost >= 0.0 and cost <= 1e100;
end PGGraphCost;
