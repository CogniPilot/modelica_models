within Meshing;
function addWallLayers "Insert conforming layers between a closed body wall and its adjacent fluid cells"
  input PolyMesh core;
  input Integer layers(min=1);
  input Real thickness(unit="m") "Total thickness into fluid";
  input Real growth(min=1)=1.2 "Thickness ratio moving away from wall";
  output PolyMesh mesh;
protected
  Integer wallFace[core.nFaces],wallPoint[core.nPoints]=fill(0,core.nPoints);
  Integer head[core.nPoints]=fill(0,core.nPoints),next[core.nCorners];
  Integer fromPoint[core.nCorners],toPoint[core.nCorners],left[core.nCorners],right[core.nCorners]=fill(0,core.nCorners);
  Real inward[core.nPoints,3]=zeros(core.nPoints,3);
  Real faceInward[core.nFaces,3];
  Integer firstIncident[core.nPoints]=fill(0,core.nPoints), nextIncident[core.nCorners], incidentFace[core.nCorners];
  Real direction[3], delta[3], candidate, least, stepLength;
  Integer incidence, incidentCount, limitingFace;
  Integer nf=0,np=0,ne=0,nc=0;
  Integer a,b,e,j2,f,faceCount,cornerCount,layerCell,innerA,innerB,outerA,outerB;
  Real normal[3],centre[3],distance;
algorithm
  assert(layers>0 and thickness>0 and growth>=1,"Invalid wall-layer dimensions");
  for i in 1:core.nFaces loop
    if core.patch[i]==7 then
      nf:=nf+1;
      wallFace[nf]:=i;
      nc:=nc+core.first[i+1]-core.first[i];
      normal:=zeros(3);
      centre:=core.points[core.corners[core.first[i]],:];
      for j in core.first[i]:core.first[i+1]-1 loop
        j2:=if j+1==core.first[i+1] then
          core.first[i] else j+1;
        a:=core.corners[j];
        b:=core.corners[j2];
        normal:=normal+cross(core.points[a,:]-centre,core.points[b,:]-centre);
        if wallPoint[a]==0 then
          np:=np+1;
          wallPoint[a]:=np;
        end if;
        e:=head[min(a,b)];
        while e<>0 loop
          if max(fromPoint[e],toPoint[e])==max(a,b) then break;end if;
          e:=next[e];
        end while;
        if e==0 then
          ne:=ne+1;
          fromPoint[ne]:=a;
          toPoint[ne]:=b;
          left[ne]:=nf;
          next[ne]:=head[min(a,b)];
          head[min(a,b)]:=ne;
        else
          assert(right[e]==0 and fromPoint[e]==b,"Body patch has a nonmanifold or inconsistently wound edge");
          right[e]:=nf;
        end if;
      end for;
      assert(normal*normal>0,"Zero-area wall face");
      faceInward[nf,:]:=-normal/sqrt(normal*normal);
      for j in core.first[i]:core.first[i+1]-1 loop
        a:=core.corners[j];
        inward[a,:]:=inward[a,:]-normal;
        nextIncident[j]:=firstIncident[a];
        firstIncident[a]:=j;
        incidentFace[j]:=nf;
      end for;
    end if;
  end for;
  assert(nf>0 and min(right[1:ne])>0,"Layers require a closed resolved body patch");
  mesh:=PolyMesh(core.nPoints+layers*np,core.nFaces+layers*(nf+ne),core.nCorners+layers*(nc+4*ne),core.nCells+layers*nf,
    zeros(core.nPoints+layers*np,3),fill(0,core.nFaces+layers*(nf+ne)+1),fill(0,core.nCorners+layers*(nc+4*ne)),
    fill(0,core.nFaces+layers*(nf+ne)),fill(0,core.nFaces+layers*(nf+ne)),fill(0,core.nFaces+layers*(nf+ne)));
  for i in 1:core.nPoints loop mesh.points[i,:]:=core.points[i,:];end for;
  for i in 1:core.nCorners loop mesh.corners[i]:=core.corners[i];end for;
  for f in 1:core.nFaces loop
    mesh.first[f]:=core.first[f];
    mesh.owner[f]:=core.owner[f];
    mesh.neighbour[f]:=core.neighbour[f];
    mesh.patch[f]:=core.patch[f];
  end for;
  for i in 1:core.nPoints loop
    if wallPoint[i]>0 then
      assert(inward[i,:]*inward[i,:]>0,"Cannot determine wall layer normal");
      inward[i,:]:=inward[i,:]/sqrt(inward[i,:]*inward[i,:]);
      // The nearest point of the convex hull of incident unit normals to
      // the origin gives the direction of greatest minimum face clearance.
      direction:=zeros(3);
      incidence:=firstIncident[i];
      incidentCount:=0;
      while incidence>0 loop
        direction:=direction+faceInward[incidentFace[incidence],:];
        incidentCount:=incidentCount+1;
        incidence:=nextIncident[incidence];
      end while;
      direction:=direction/incidentCount;
      for iteration in 1:60 loop
        incidence:=firstIncident[i];
        least:=Modelica.Constants.inf;
        limitingFace:=incidentFace[incidence];
        while incidence>0 loop
          f:=incidentFace[incidence];
          candidate:=direction*faceInward[f,:];
          if candidate<least then
            least:=candidate;
            limitingFace:=f;
          end if;
          incidence:=nextIncident[incidence];
        end while;
        delta:=faceInward[limitingFace,:]-direction;
        stepLength:=min(1,max(0,-(direction*delta)/max(1e-30,delta*delta)));
        if stepLength<1e-10 then break;end if;
        direction:=direction+stepLength*delta;
      end for;
      // Retain the area-weighted direction on smooth parts of the wall.
      assert(direction*direction>1e-20,"No common extrusion direction at wall vertex "+String(i));
      direction:=0.6*inward[i,:]+0.4*direction/sqrt(direction*direction);
      inward[i,:]:=direction/sqrt(direction*direction);
      mesh.points[i,:]:=core.points[i,:]+thickness*inward[i,:];
      for k in 1:layers loop
        distance:=if growth==1 then
          thickness*(layers-k)/layers else thickness*(growth^(layers-k)-1)/(growth^layers-1);
        mesh.points[core.nPoints+(k-1)*np+wallPoint[i],:]:=core.points[i,:]+distance*inward[i,:];
      end for;
    end if;
  end for;
  faceCount:=core.nFaces;
  cornerCount:=core.nCorners;
  for k in 1:layers loop
    for i in 1:nf loop
      f:=wallFace[i];
      layerCell:=core.nCells+(k-1)*nf+i;
      if k==1 then
        mesh.patch[f]:=0;
        mesh.neighbour[f]:=layerCell;
      end if;
      faceCount:=faceCount+1;
      mesh.first[faceCount]:=cornerCount+1;
      mesh.owner[faceCount]:=layerCell;
      mesh.neighbour[faceCount]:=if k==layers then
        0 else layerCell+nf;
      mesh.patch[faceCount]:=if k==layers then
        7 else 0;
      for j in core.first[f]:core.first[f+1]-1 loop
        cornerCount:=cornerCount+1;
        mesh.corners[cornerCount]:=core.nPoints+(k-1)*np+wallPoint[core.corners[j]];
      end for;
    end for;
    for edge in 1:ne loop
      a:=fromPoint[edge];
      b:=toPoint[edge];
      outerA:=core.nPoints+(k-1)*np+wallPoint[a];
      outerB:=core.nPoints+(k-1)*np+wallPoint[b];
      innerA:=if k==1 then
        a else outerA-np;
      innerB:=if k==1 then
        b else outerB-np;
      faceCount:=faceCount+1;
      mesh.first[faceCount]:=cornerCount+1;
      mesh.owner[faceCount]:=core.nCells+(k-1)*nf+left[edge];
      mesh.neighbour[faceCount]:=core.nCells+(k-1)*nf+right[edge];
      mesh.corners[cornerCount+1]:=innerA;
      mesh.corners[cornerCount+2]:=innerB;
      mesh.corners[cornerCount+3]:=outerB;
      mesh.corners[cornerCount+4]:=outerA;
      cornerCount:=cornerCount+4;
    end for;
  end for;
  mesh.first[faceCount+1]:=cornerCount+1;
end addWallLayers;
