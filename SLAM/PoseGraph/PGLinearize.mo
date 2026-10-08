within SLAM.PoseGraph;
function PGLinearize
  import PGEdge = SLAM.PoseGraph.PGEdge;

  input Real p[:,3]; input Real R[size(p,1),3,3]; input Real edgeMask[:];
  input Integer source[size(edgeMask,1)]; input Integer target[size(edgeMask,1)];
  input Real translation[size(edgeMask,1),3]; input Real measuredRotation[size(edgeMask,1),3,3];
  input Real information[size(edgeMask,1),6,6];
  output Real Ji[size(edgeMask,1),6,6]; output Real Jj[size(edgeMask,1),6,6];
  output Real gradient[size(p,1),6]; output Real blocks[size(p,1),6,6];
protected Integer i; Integer j; Real residual[6]; Boolean valid;
  Real edgeJi[6,6]; Real edgeJj[6,6];
algorithm
  Ji := zeros(size(edgeMask,1),6,6); Jj := zeros(size(edgeMask,1),6,6);
  gradient := zeros(size(p,1),6); blocks := zeros(size(p,1),6,6); residual := zeros(6); valid := false; i := 1; j := 1;
  edgeJi := zeros(6,6); edgeJj := zeros(6,6);
  for edge in 1:size(edgeMask,1) loop
    if edgeMask[edge] == 1.0 then
      i := source[edge]; j := target[edge];
      (residual,edgeJi,edgeJj,valid) := PGEdge(p[i,:],R[i,:,:],p[j,:],R[j,:,:],translation[edge,:],measuredRotation[edge,:,:]);
      Ji[edge,:,:] := edgeJi; Jj[edge,:,:] := edgeJj;
      if i > 1 then
        gradient[i,:] := gradient[i,:]+transpose(Ji[edge,:,:])*(information[edge,:,:]*residual);
        blocks[i,:,:] := blocks[i,:,:]+transpose(Ji[edge,:,:])*information[edge,:,:]*Ji[edge,:,:];
      end if;
      if j > 1 then
        gradient[j,:] := gradient[j,:]+transpose(Jj[edge,:,:])*(information[edge,:,:]*residual);
        blocks[j,:,:] := blocks[j,:,:]+transpose(Jj[edge,:,:])*information[edge,:,:]*Jj[edge,:,:];
      end if;
    end if;
  end for;
end PGLinearize;
