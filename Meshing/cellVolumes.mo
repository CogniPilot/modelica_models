within Meshing;
function cellVolumes "Compute signed cell volumes and check the face orientation and cell closure"
  input PolyMesh mesh;
  output Real volumes[mesh.nCells](each unit="m3");
protected
  Real closure[mesh.nCells,3]=zeros(mesh.nCells,3),areaSum[mesh.nCells]=zeros(mesh.nCells);
  Real centres[mesh.nCells,3]=zeros(mesh.nCells,3),centre[3],area[3],a[3],b[3];
  Integer count[mesh.nCells]=fill(0,mesh.nCells),own,other,next;
algorithm
  volumes:=zeros(mesh.nCells);
  for f in 1:mesh.nFaces loop
    own:=mesh.owner[f];
    other:=mesh.neighbour[f];
    centre:=zeros(3);
    for j in mesh.first[f]:mesh.first[f+1]-1 loop
      centre:=centre+mesh.points[mesh.corners[j],:];
    end for;
    centre:=centre/(mesh.first[f+1]-mesh.first[f]);
    centres[own,:]:=centres[own,:]+centre;
    count[own]:=count[own]+1;
    if other>0 then
      centres[other,:]:=centres[other,:]+centre;
      count[other]:=count[other]+1;
    end if;
  end for;
  for c in 1:mesh.nCells loop
    assert(count[c]>=4,"Cell has fewer than four faces");centres[c,:]:=centres[c,:]/count[c];
  end for;
  for f in 1:mesh.nFaces loop
    own:=mesh.owner[f];
    other:=mesh.neighbour[f];
    centre:=zeros(3);
    for j in mesh.first[f]:mesh.first[f+1]-1 loop
      centre:=centre+mesh.points[mesh.corners[j],:];
    end for;
    centre:=centre/(mesh.first[f+1]-mesh.first[f]);
    for j in mesh.first[f]:mesh.first[f+1]-1 loop
      next:=if j+1==mesh.first[f+1] then
        mesh.first[f] else j+1;
      a:=mesh.points[mesh.corners[j],:]-centre;
      b:=mesh.points[mesh.corners[next],:]-centre;
      area:=0.5*cross(a,b);
      volumes[own]:=volumes[own]+(centre-centres[own,:])*area/3;
      closure[own,:]:=closure[own,:]+area;
      areaSum[own]:=areaSum[own]+sqrt(area*area);
      if other>0 then
        volumes[other]:=volumes[other]-(centre-centres[other,:])*area/3;
        closure[other,:]:=closure[other,:]-area;
        areaSum[other]:=areaSum[other]+sqrt(area*area);
      end if;
    end for;
  end for;
  for c in 1:mesh.nCells loop
    assert(volumes[c]>0,"Inverted or zero-volume cell "+String(c));
    assert(sqrt(closure[c,:]*closure[c,:])<1e-8*areaSum[c],"Unclosed cell "+String(c));
  end for;
end cellVolumes;
