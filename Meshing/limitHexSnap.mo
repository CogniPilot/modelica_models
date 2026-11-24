within Meshing;
function limitHexSnap "Backtrack shared wall vertices when a snapped hexahedron loses face-plane clearance"
  input PolyMesh core "Unsnapped Cartesian mesh";
  input Real candidate[core.nPoints,3](each unit="m");
  output Real points[core.nPoints,3](each unit="m");
protected
  Integer faces[core.nCells,6],count[core.nCells]=fill(0,core.nCells),active[core.nCells],nActive=0;
  Integer cell,face,corner,point;
  Boolean moved[core.nPoints],affected[core.nCells]=fill(false,core.nCells);
  Boolean quad[core.nCells]=fill(true,core.nCells),reduce[core.nPoints],invalid,changed;
  Real centres[6,3],normals[6,3],v[4,3],direction[3],distance,offset[3],scale,t;
  Real fraction[core.nPoints]=ones(core.nPoints);
algorithm
  points:=candidate;
  for i in 1:core.nPoints loop
    moved[i]:=max(abs(candidate[i,:]-core.points[i,:]))>0;
  end for;
  for f in 1:core.nFaces loop
    for side in 1:(if core.neighbour[f]>0 then 2 else 1) loop
      cell:=if side==1 then core.owner[f] else core.neighbour[f];
      count[cell]:=count[cell]+1;
      if count[cell]<=6 then faces[cell,count[cell]]:=f;end if;
      quad[cell]:=quad[cell] and core.first[f+1]-core.first[f]==4;
      for j in core.first[f]:core.first[f+1]-1 loop
        if moved[core.corners[j]] then affected[cell]:=true;end if;
      end for;
    end for;
  end for;
  for c in 1:core.nCells loop
    if affected[c] and quad[c] and count[c]==6 then
      nActive:=nActive+1;
      active[nActive]:=c;
    end if;
  end for;
  changed:=true;
  while changed loop
    changed:=false;
    reduce:=fill(false,core.nPoints);
    for i in 1:nActive loop
      cell:=active[i];
      for k in 1:6 loop
        face:=faces[cell,k];
        for j in 1:4 loop v[j,:]:=points[core.corners[core.first[face]+j-1],:];end for;
        normals[k,:]:=cross(v[3,:]-v[1,:],v[4,:]-v[2,:]);
        centres[k,:]:=(v[1,:]+v[2,:]+v[3,:]+v[4,:])/4;
        scale:=sqrt(normals[k,:]*normals[k,:]);
        if scale>0 then
          // Area-weighted centroid of the four triangles about the vertex average.
          normals[k,:]:=normals[k,:]/scale;
          offset:=zeros(3);
          for j in 1:4 loop
            corner:=mod(j,4)+1;
            t:=normals[k,:]*cross(v[j,:]-centres[k,:],v[corner,:]-centres[k,:]);
            offset:=offset+t*((v[j,:]-centres[k,:])+(v[corner,:]-centres[k,:]));
          end for;
          centres[k,:]:=centres[k,:]+offset/(3*scale);
          if core.owner[face]<>cell then normals[k,:]:=-normals[k,:];end if;
        end if;
      end for;
      invalid:=false;
      for k in 1:6 loop
        for j in 1:6 loop
          if j<>k then
            direction:=centres[j,:]-centres[k,:];
            distance:=sqrt(direction*direction);
            if normals[k,:]*direction>=-0.01*distance then invalid:=true;break;end if;
          end if;
        end for;
        if invalid then break;end if;
      end for;
      if invalid then
        for k in 1:6 loop
          face:=faces[cell,k];
          for j in core.first[face]:core.first[face+1]-1 loop
            point:=core.corners[j];
            if moved[point] and fraction[point]>0 then reduce[point]:=true;changed:=true;end if;
          end for;
        end for;
      end if;
    end for;
    if changed then
      for p in 1:core.nPoints loop
        if reduce[p] then
          fraction[p]:=if fraction[p]<0.1 then 0 else 0.75*fraction[p];
          points[p,:]:=core.points[p,:]+fraction[p]*(candidate[p,:]-core.points[p,:]);
        end if;
      end for;
    end if;
  end while;
end limitHexSnap;
