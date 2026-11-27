within Meshing;
function clipTetrahedra "Keep the positive side of a linear nodal level set on a conforming tetrahedral mesh"
  input Real points[:,3](each unit="m");
  input Integer tetrahedra[:,4];
  input Real levelSet[size(points,1)] "Positive in the retained material, zero on the interface";
  input Integer boundaryPatch[size(tetrahedra,1),4] "Face IDs opposite each local tetrahedron vertex";
  input Integer interfacePatch(min=1)=7;
  output PolyMesh mesh;
protected
  PolyMesh background=fromTetrahedra(points,tetrahedra,boundaryPatch);
  Integer nt=size(tetrahedra,1),np=size(points,1),nf=background.nFaces,cap=6*nt;
  Integer cellMap[nt]=fill(0,nt),pointMap[np]=fill(0,np);
  Integer head[np]=fill(0,np),link[cap],endPoint[cap],edgePoint[cap],edgeCount=0;
  Integer cutFrom[nt,4],cutTo[nt,4],cutCount[nt]=fill(0,nt);
  Real vertices[np+cap,3],alpha,value;
  Boolean positive,negative;
  Integer first[nf+nt+1],corners[4*(nf+nt)],owner[nf+nt],other[nf+nt],patch[nf+nt];
  Integer nc=0,nv=0,faces=0,nk=0,own,nei,sideCell,n,polygon[4],a,b,lo,hi,e;
  Integer zeroEdge,u,v,origin,nextVertex;
  Boolean onCut[4],mixed[nt],reverseFace,allZero;
algorithm
  assert(interfacePatch>0,"Interface patch ID must be positive");
  for c in 1:nt loop
    positive:=false;
    negative:=false;
    for j in 1:4 loop
      value:=levelSet[tetrahedra[c,j]];
      positive:=positive or value>0;
      negative:=negative or value<0;
    end for;
    mixed[c]:=positive and negative;
    if positive then nc:=nc+1;cellMap[c]:=nc;end if;
  end for;
  assert(nc>0,"Level set contains no fluid tetrahedra");
  for f in 1:nf loop
    own:=background.owner[f];
    nei:=background.neighbour[f];
    if cellMap[own]>0 or (if nei>0 then cellMap[nei]>0 else false) then
      n:=0;
      for j in 0:2 loop
        a:=background.corners[background.first[f]+j];
        b:=background.corners[background.first[f]+mod(j+1,3)];
        if levelSet[a]>=0 then
          if pointMap[a]==0 then
            nv:=nv+1;vertices[nv,:]:=points[a,:];pointMap[a]:=nv;
          end if;
          n:=n+1;polygon[n]:=pointMap[a];onCut[n]:=levelSet[a]==0;
        end if;
        if (levelSet[a]<0 and levelSet[b]>0) or (levelSet[b]<0 and levelSet[a]>0) then
          lo:=min(a,b);hi:=max(a,b);e:=head[lo];
          while e>0 loop
            if endPoint[e]==hi then break;end if;
            e:=link[e];
          end while;
          if e==0 then
            alpha:=levelSet[lo]/(levelSet[lo]-levelSet[hi]);
            nv:=nv+1;vertices[nv,:]:=points[lo,:]+alpha*(points[hi,:]-points[lo,:]);
            edgeCount:=edgeCount+1;e:=edgeCount;link[e]:=head[lo];head[lo]:=e;
            endPoint[e]:=hi;edgePoint[e]:=nv;
          end if;
          n:=n+1;polygon[n]:=edgePoint[e];onCut[n]:=true;
        end if;
      end for;
      if n>=3 then
        reverseFace:=cellMap[own]==0;
        faces:=faces+1;first[faces]:=nk+1;
        owner[faces]:=if reverseFace then cellMap[nei] else cellMap[own];
        other[faces]:=if reverseFace or nei==0 then 0 else cellMap[nei];
        patch[faces]:=if nei==0 then background.patch[f] elseif other[faces]==0 then interfacePatch else 0;
        allZero:=true;
        for j in 1:n loop
          nk:=nk+1;corners[nk]:=polygon[if reverseFace then n-j+1 else j];
          allZero:=allZero and onCut[j];
        end for;
        if not allZero then
          for j in 1:n loop
            zeroEdge:=mod(j,n)+1;
            if onCut[j] and onCut[zeroEdge] then
              for side in 1:(if nei==0 then 1 else 2) loop
                sideCell:=if side==1 then own else nei;
                if mixed[sideCell] then
                  cutCount[sideCell]:=cutCount[sideCell]+1;
                  u:=if side==1 then polygon[zeroEdge] else polygon[j];
                  v:=if side==1 then polygon[j] else polygon[zeroEdge];
                  cutFrom[sideCell,cutCount[sideCell]]:=u;cutTo[sideCell,cutCount[sideCell]]:=v;
                end if;
              end for;
            end if;
          end for;
        end if;
      end if;
    end if;
  end for;
  // The cut segments on the oriented tetrahedron faces form one closed cap.
  for c in 1:nt loop
    if mixed[c] then
      n:=cutCount[c];assert(n==3 or n==4,"Incomplete cut polygon");
      faces:=faces+1;first[faces]:=nk+1;owner[faces]:=cellMap[c];other[faces]:=0;patch[faces]:=interfacePatch;
      origin:=cutFrom[c,1];u:=origin;
      for j in 1:n loop
        nk:=nk+1;corners[nk]:=u;nextVertex:=0;
        for s in 1:n loop
          if cutFrom[c,s]==u then nextVertex:=cutTo[c,s];break;end if;
        end for;
        assert(nextVertex>0,"Disconnected cut polygon");u:=nextVertex;
      end for;
      assert(u==origin,"Open cut polygon");
    end if;
  end for;
  first[faces+1]:=nk+1;
  mesh:=PolyMesh(nv,faces,nk,nc,zeros(nv,3),fill(0,faces+1),fill(0,nk),
    fill(0,faces),fill(0,faces),fill(0,faces));
  for i in 1:nv loop
    for d in 1:3 loop mesh.points[i,d]:=vertices[i,d];end for;
  end for;
  for f in 1:faces loop
    mesh.first[f]:=first[f];
    mesh.owner[f]:=owner[f];
    mesh.neighbour[f]:=other[f];
    mesh.patch[f]:=patch[f];
  end for;
  mesh.first[faces+1]:=nk+1;
  for j in 1:nk loop mesh.corners[j]:=corners[j];end for;
end clipTetrahedra;
