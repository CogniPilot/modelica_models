within Meshing;
function cellVolumes "Compute signed cell volumes and check the face orientation and cell closure"
  input PolyMesh mesh;
  input Boolean checkFacePyramids=false "Require each face to form a positive signed pyramid with the cell volume centroid";
  output Real volumes[mesh.nCells](each unit="m3");
protected
  Real closure[mesh.nCells,3]=zeros(mesh.nCells,3),areaSum[mesh.nCells]=zeros(mesh.nCells);
  Real centres[mesh.nCells,3]=zeros(mesh.nCells,3),centre[3],area[3],a[3],b[3];
  Real firstMoment[if checkFacePyramids then mesh.nCells else 0,3];
  Real faceCentre[if checkFacePyramids then mesh.nFaces else 0,3],faceArea[if checkFacePyramids then mesh.nFaces else 0,3];
  Real displacement[3],volume;
  Integer count[mesh.nCells]=fill(0,mesh.nCells),own,other,next;
algorithm
  volumes:=zeros(mesh.nCells);
  if checkFacePyramids then firstMoment:=zeros(mesh.nCells,3);end if;
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
    if checkFacePyramids then
      faceCentre[f,:]:=centre;
      faceArea[f,:]:=zeros(3);
    end if;
    for j in mesh.first[f]:mesh.first[f+1]-1 loop
      next:=if j+1==mesh.first[f+1] then
        mesh.first[f] else j+1;
      a:=mesh.points[mesh.corners[j],:]-centre;
      b:=mesh.points[mesh.corners[next],:]-centre;
      area:=0.5*cross(a,b);
      displacement:=centre-centres[own,:];
      volume:=displacement*area/3;
      volumes[own]:=volumes[own]+volume;
      if checkFacePyramids then
        faceArea[f,:]:=faceArea[f,:]+area;
        firstMoment[own,:]:=firstMoment[own,:]+(volume/4)*(3*displacement+a+b);
      end if;
      closure[own,:]:=closure[own,:]+area;
      areaSum[own]:=areaSum[own]+sqrt(area*area);
      if other>0 then
        displacement:=centre-centres[other,:];
        volume:=-displacement*area/3;
        volumes[other]:=volumes[other]+volume;
        if checkFacePyramids then
          firstMoment[other,:]:=firstMoment[other,:]+(volume/4)*(3*displacement+a+b);
        end if;
        closure[other,:]:=closure[other,:]-area;
        areaSum[other]:=areaSum[other]+sqrt(area*area);
      end if;
    end for;
  end for;
  for c in 1:mesh.nCells loop
    assert(volumes[c]>0,"Inverted or zero-volume cell "+String(c));
    assert(sqrt(closure[c,:]*closure[c,:])<1e-8*areaSum[c],"Unclosed cell "+String(c));
    if checkFacePyramids then centres[c,:]:=centres[c,:]+firstMoment[c,:]/volumes[c];end if;
  end for;
  if checkFacePyramids then
    for f in 1:mesh.nFaces loop
      own:=mesh.owner[f];
      other:=mesh.neighbour[f];
      assert((faceCentre[f,:]-centres[own,:])*faceArea[f,:]>0,
        "Inverted face pyramid at face "+String(f)+" in owner cell "+String(own)+". Check resolution, snapping and layer settings.");
      if other>0 then
        assert((centres[other,:]-faceCentre[f,:])*faceArea[f,:]>0,
          "Inverted face pyramid at face "+String(f)+" in neighbour cell "+String(other)+". Check resolution, snapping and layer settings.");
      end if;
    end for;
  end if;
end cellVolumes;
