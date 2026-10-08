within SLAM.PoseGraph;
function PGValidateGraph
  import PGCholesky = SLAM.PoseGraph.PGCholesky;
  import PGProperRotation = SLAM.PoseGraph.PGProperRotation;

  input Real p[:,3]; input Real R[size(p,1),3,3]; input Real nodeMask[size(p,1)]; input Real edgeMask[:];
  input Real fromNode[size(edgeMask,1)]; input Real toNode[size(edgeMask,1)];
  input Real translation[size(edgeMask,1),3]; input Real measuredRotation[size(edgeMask,1),3,3];
  input Real information[size(edgeMask,1),6,6];
  output Integer source[size(edgeMask,1)]; output Integer target[size(edgeMask,1)];
  output Real status; output Real activeNodes; output Real activeEdges;
protected Boolean valid; Boolean proper; Boolean endpoints; Boolean factorValid;
  Real L[6,6]; Real reached[size(p,1)]; Integer i; Integer j;
algorithm
  source := fill(1,size(edgeMask,1)); target := fill(1,size(edgeMask,1)); reached := zeros(size(p,1));
  valid := size(p,1) >= 1 and size(p,1) <= 128 and size(edgeMask,1) >= 1 and size(edgeMask,1) <= 256 and nodeMask[1] == 1.0;
  proper := false; endpoints := false; factorValid := false; L := zeros(6,6); i := 1; j := 1;
  status := -1.0; activeNodes := 0.0; activeEdges := 0.0;
  for node in 1:size(p,1) loop
    valid := valid and (nodeMask[node] == 0.0 or nodeMask[node] == 1.0);
    for k in 1:3 loop valid := valid and (nodeMask[node] == 0.0 or abs(p[node,k]) <= 1e6); end for;
    proper := PGProperRotation(R[node,:,:]); valid := valid and (nodeMask[node] == 0.0 or proper);
    activeNodes := activeNodes+(if nodeMask[node] == 1.0 then 1.0 else 0.0);
  end for;
  status := if valid then -2.0 else status;
  for edge in 1:size(edgeMask,1) loop
    endpoints := edgeMask[edge] == 1.0 and fromNode[edge] >= 1.0 and fromNode[edge] <= size(p,1)
      and floor(fromNode[edge]) == fromNode[edge] and toNode[edge] >= 1.0 and toNode[edge] <= size(p,1)
      and floor(toNode[edge]) == toNode[edge] and fromNode[edge] <> toNode[edge];
    valid := valid and (edgeMask[edge] == 0.0 or (edgeMask[edge] == 1.0 and endpoints));
    source[edge] := if endpoints then integer(fromNode[edge]) else 1;
    target[edge] := if endpoints then integer(toNode[edge]) else 1;
    valid := valid and (edgeMask[edge] == 0.0 or (nodeMask[source[edge]] == 1.0 and nodeMask[target[edge]] == 1.0));
    for k in 1:3 loop valid := valid and (edgeMask[edge] == 0.0 or abs(translation[edge,k]) <= 1e6); end for;
    proper := PGProperRotation(measuredRotation[edge,:,:]);
    (L,factorValid) := PGCholesky(information[edge,:,:],1e-10);
    valid := valid and (edgeMask[edge] == 0.0 or (proper and factorValid));
    activeEdges := activeEdges+(if edgeMask[edge] == 1.0 then 1.0 else 0.0);
  end for;
  status := if valid then -3.0 else status; reached[1] := 1.0;
  for pass in 1:size(p,1) loop
    for edge in 1:size(edgeMask,1) loop
      if valid and edgeMask[edge] == 1.0 then
        i := source[edge]; j := target[edge];
        if reached[i] == 1.0 or reached[j] == 1.0 then reached[i] := 1.0; reached[j] := 1.0; end if;
      end if;
    end for;
  end for;
  for node in 1:size(p,1) loop valid := valid and (nodeMask[node] == 0.0 or reached[node] == 1.0); end for;
  status := if valid then 1.0 else status;
end PGValidateGraph;
