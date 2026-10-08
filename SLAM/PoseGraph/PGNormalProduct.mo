within SLAM.PoseGraph;
function PGNormalProduct
  input Real x[:,6]; input Real nodeMask[size(x,1)];
  input Real edgeMask[:]; input Integer source[size(edgeMask,1)]; input Integer target[size(edgeMask,1)];
  input Real Ji[size(edgeMask,1),6,6]; input Real Jj[size(edgeMask,1),6,6];
  input Real information[size(edgeMask,1),6,6]; input Real dampingDiagonal[size(x,1),6]; input Real damping;
  output Real y[size(x,1),6];
protected Integer i; Integer j; Real row[6]; Real weighted[6];
algorithm
  y := zeros(size(x,1),6); i := 1; j := 1; row := zeros(6); weighted := zeros(6);
  for node in 1:size(x,1) loop
    for k in 1:6 loop
      y[node,k] := if node > 1 and nodeMask[node] == 1.0 then damping*dampingDiagonal[node,k]*x[node,k] else 0.0;
    end for;
  end for;
  for edge in 1:size(edgeMask,1) loop
    if edgeMask[edge] == 1.0 then
      i := source[edge]; j := target[edge];
      row := Ji[edge,:,:]*x[i,:]+Jj[edge,:,:]*x[j,:]; weighted := information[edge,:,:]*row;
      if i > 1 then y[i,:] := y[i,:]+transpose(Ji[edge,:,:])*weighted; end if;
      if j > 1 then y[j,:] := y[j,:]+transpose(Jj[edge,:,:])*weighted; end if;
    end if;
  end for;
  // Node1 is the exact gauge: callers keep x[1,:]=0; its operator row is zero.
end PGNormalProduct;
