within Meshing;
function surfacePrismMesh "Extrude a closed triangular wall into a blade-fitted near-wall prism block"
  input Real points[:,3](each unit="m") "Referenced surface vertices";
  input Integer triangles[:,3] "Closed, consistently outward winding, one-based indices";
  input Real heights[:](each unit="m") "Cumulative outward distances from the wall";
  input Integer wallPatch(min=1)=7,interfacePatch(min=1)=8;
  input Boolean limitThickness=false "Locally cap total thickness at one twentieth of the smallest incident triangle altitude";
  output PolyMesh mesh;
protected
  Integer np=size(points,1),nf=size(triangles,1),nl=size(heights,1),cap=3*nf,ne=0;
  Integer head[np]=fill(0,np),next[cap],fromId[cap],toId[cap],left[cap],right[cap]=fill(0,cap);
  Real thicknessScale[np]=ones(np),doubleArea,oppositeEdge[3];
  Real faceNormals[nf,3],projection;
  Boolean adjusted;
  Real normals[np,3]=zeros(np,3),u[3],v[3],normal[3],angle;
  Integer a,b,e,newFace=0,corner=0,layerBase;
  Real volumes[:];
algorithm
  assert(np>=4 and nf>=4 and nl>=1 and min(triangles)>=1 and max(triangles)<=np,"Invalid indexed wall or missing layers");
  assert(wallPatch>0 and interfacePatch>0 and wallPatch<>interfacePatch,"Choose distinct positive wall and interface IDs");
  for k in 1:nl loop
    assert(heights[k]>(if k==1 then 0 else heights[k-1]),"Layer distances must increase from zero");
  end for;
  for f in 1:nf loop
    u:=points[triangles[f,2],:]-points[triangles[f,1],:];
    v:=points[triangles[f,3],:]-points[triangles[f,1],:];normal:=cross(u,v);
    doubleArea:=sqrt(normal*normal);
    assert(doubleArea>0,"Degenerate triangle");normal:=normal/doubleArea;
    faceNormals[f,:]:=normal;
    for j in 1:3 loop
      a:=triangles[f,j];b:=triangles[f,mod(j,3)+1];
      u:=points[b,:]-points[a,:];v:=points[triangles[f,mod(j+1,3)+1],:]-points[a,:];
      if limitThickness then
        oppositeEdge:=u-v;
        thicknessScale[a]:=min(thicknessScale[a],0.05*doubleArea/(sqrt(oppositeEdge*oppositeEdge)*heights[nl]));
      end if;
      angle:=Modelica.Math.atan2(sqrt(cross(u,v)*cross(u,v)),u*v);
      normals[a,:]:=normals[a,:]+angle*normal;
      e:=head[min(a,b)];
      while e>0 loop
        if max(a,b)==max(fromId[e],toId[e]) then break;end if;
        e:=next[e];
      end while;
      if e==0 then
        ne:=ne+1;e:=ne;fromId[e]:=a;toId[e]:=b;left[e]:=f;next[e]:=head[min(a,b)];head[min(a,b)]:=e;
      else
        assert(fromId[e]==b and right[e]==0,"Nonmanifold or inconsistently wound wall");
        right[e]:=f;
      end if;
    end for;
  end for;
  assert(min(right[1:ne])>0,"Open edge in prism wall");
  // At a sharply folded vertex the angle-weighted direction can enter an incident facet.
  // Project such directions onto the common outward half-spaces before extrusion.
  for i in 1:np loop
    assert(normals[i,:]*normals[i,:]>0,"Unreferenced or singular wall vertex");
    normals[i,:]:=normals[i,:]/sqrt(normals[i,:]*normals[i,:]);
  end for;
  for iteration in 1:128 loop
    adjusted:=false;
    for f in 1:nf loop
      for j in 1:3 loop
        a:=triangles[f,j];
        projection:=normals[a,:]*faceNormals[f,:];
        if projection<0.1 then
          normal:=normals[a,:]+(0.25-projection)*faceNormals[f,:];
          normals[a,:]:=normal/sqrt(normal*normal);
          adjusted:=true;
        end if;
      end for;
    end for;
    if not adjusted then break;end if;
  end for;
  assert(not adjusted,"No common outward extrusion direction at the requested angular margin");
  mesh:=PolyMesh((nl+1)*np,(nl+1)*nf+nl*ne,(nl+1)*3*nf+4*nl*ne,nl*nf,
    zeros((nl+1)*np,3),fill(0,(nl+1)*nf+nl*ne+1),fill(0,(nl+1)*3*nf+4*nl*ne),
    fill(0,(nl+1)*nf+nl*ne),fill(0,(nl+1)*nf+nl*ne),fill(0,(nl+1)*nf+nl*ne));
  for i in 1:np loop
    normal:=normals[i,:];
    mesh.points[i,:]:=points[i,:];
    for k in 1:nl loop mesh.points[np*k+i,:]:=points[i,:]+heights[k]*thicknessScale[i]*normal;end for;
  end for;
  for k in 0:nl loop
    for f in 1:nf loop
      newFace:=newFace+1;mesh.first[newFace]:=corner+1;
      mesh.owner[newFace]:=max(0,k-1)*nf+f;
      mesh.neighbour[newFace]:=if k>0 and k<nl then k*nf+f else 0;
      mesh.patch[newFace]:=if k==0 then wallPatch elseif k==nl then interfacePatch else 0;
      for j in 1:3 loop
        corner:=corner+1;mesh.corners[corner]:=k*np+triangles[f,if k==0 then 4-j else j];
      end for;
    end for;
  end for;
  for k in 1:nl loop
    layerBase:=(k-1)*np;
    for edge in 1:ne loop
      newFace:=newFace+1;mesh.first[newFace]:=corner+1;
      mesh.owner[newFace]:=(k-1)*nf+left[edge];mesh.neighbour[newFace]:=(k-1)*nf+right[edge];
      a:=layerBase+fromId[edge];b:=layerBase+toId[edge];
      mesh.corners[corner+1:corner+4]:={a,b,b+np,a+np};corner:=corner+4;
    end for;
  end for;
  mesh.first[newFace+1]:=corner+1;
  volumes:=cellVolumes(mesh,true);
  assert(min(volumes)>0,"Invalid extrusion; reduce the wall-layer thickness");
end surfacePrismMesh;
