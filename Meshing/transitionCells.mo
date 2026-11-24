within Meshing;
function transitionCells "Join refinement levels using pyramids and tetrahedra around a coarse-cell centre"
  input PolyMesh core "Hex-dominant Cartesian mesh with conforming split interfaces";
  output PolyMesh mesh;
protected
  Boolean split[core.nCells]=fill(false,core.nCells);
  Boolean wallSide[core.nCells,6]=fill(false,core.nCells,6);
  Integer axis,wall;
  Integer faceCount[core.nCells]=fill(0,core.nCells), children[core.nCells]=fill(0,core.nCells);
  Integer nextCell[core.nCells],apex[core.nCells],midpoint[core.nFaces]=fill(0,core.nFaces);
  Real centre[core.nCells,3]=zeros(core.nCells,3),weight[core.nCells]=zeros(core.nCells);
  Real fc[3],area[3],a[3],b[3],w;
  Integer nPoints=core.nPoints,nCells=0,nBase=0,baseCorners=0,sideIncidences=0;
  Integer degree,pieces,vertices[4],nv,c,own,other,child,orig,j2,u,v,index,where,slot;
  Integer tableSize,head[:],link[:],parent[:],baseFace=0,sideCount=0,corner=0;
algorithm
  // A coarse cell with a split surface has several neighbours on one side.
  // Also split the cells across any face whose perimeter contains a hanging node.
  for f in 1:core.nFaces loop
    degree:=core.first[f+1]-core.first[f];
    // A single voxel across a resolved slot has no interior neighbours
    // in its wall-normal direction. Give both walls their own volume.
    if core.patch[f]==7 then
      area:=zeros(3);
      a:=core.points[core.corners[core.first[f]],:];
      for j in core.first[f]:core.first[f+1]-1 loop
        j2:=if j+1==core.first[f+1] then core.first[f] else j+1;
        area:=area+cross(core.points[core.corners[j],:]-a,core.points[core.corners[j2],:]-a);
      end for;
      axis:=if abs(area[1])>=max(abs(area[2]),abs(area[3])) then 1 elseif abs(area[2])>=abs(area[3]) then 2 else 3;
      wall:=2*axis-(if area[axis]>0 then 0 else 1);
      c:=core.owner[f];
      wallSide[c,wall]:=true;
      if wallSide[c,2*axis] and wallSide[c,2*axis-1] then split[c]:=true;end if;
    end if;
    for side in 1:(if core.neighbour[f]>0 then 2 else 1) loop
      c:=if side==1 then core.owner[f] else core.neighbour[f];
      faceCount[c]:=faceCount[c]+1;
      if degree>4 or faceCount[c]>6 then split[c]:=true;end if;
    end for;
  end for;
  if sum(if s then 1 else 0 for s in split)==0 then mesh:=core;return;end if;
  for f in 1:core.nFaces loop
    degree:=core.first[f+1]-core.first[f];
    pieces:=if degree>4 then degree else 1;
    nv:=if degree>4 then 3 else degree;
    nBase:=nBase+pieces;
    baseCorners:=baseCorners+pieces*nv;
    if degree>4 then
      nPoints:=nPoints+1;
      midpoint[f]:=nPoints;
    end if;
    for side in 1:(if core.neighbour[f]>0 then 2 else 1) loop
      c:=if side==1 then core.owner[f] else core.neighbour[f];
      if split[c] then
        children[c]:=children[c]+pieces;
        sideIncidences:=sideIncidences+pieces*nv;
      end if;
    end for;
  end for;
  for cell in 1:core.nCells loop
    nextCell[cell]:=nCells+1;
    nCells:=nCells+max(1,children[cell]);
    if split[cell] then
      nPoints:=nPoints+1;
      apex[cell]:=nPoints;
    end if;
  end for;
  tableSize:=div(sideIncidences,2);
  assert(mod(sideIncidences,2)==0,"Unpaired refinement transition faces");
  mesh:=PolyMesh(nPoints,nBase+tableSize,baseCorners+3*tableSize,nCells,zeros(nPoints,3),
    fill(0,nBase+tableSize+1),fill(0,baseCorners+3*tableSize),fill(0,nBase+tableSize),
    fill(0,nBase+tableSize),fill(0,nBase+tableSize));
  for i in 1:core.nPoints loop mesh.points[i,:]:=core.points[i,:];end for;
  head:=fill(0,tableSize);
  link:=fill(0,tableSize);
  parent:=fill(0,tableSize);
  for f in 1:core.nFaces loop
    degree:=core.first[f+1]-core.first[f];
    fc:=zeros(3);
    area:=zeros(3);
    a:=core.points[core.corners[core.first[f]],:];
    for j in core.first[f]:core.first[f+1]-1 loop
      j2:=if j+1==core.first[f+1] then core.first[f] else j+1;
      b:=core.points[core.corners[j],:];
      fc:=fc+b;
      area:=area+cross(b-a,core.points[core.corners[j2],:]-a);
    end for;
    fc:=fc/degree;
    w:=sqrt(area*area);
    if degree>4 then mesh.points[midpoint[f],:]:=fc;end if;
    own:=core.owner[f];
    other:=core.neighbour[f];
    for side in 1:(if other>0 then 2 else 1) loop
      c:=if side==1 then own else other;
      if split[c] then
        centre[c,:]:=centre[c,:]+w*fc;
        weight[c]:=weight[c]+w;
      end if;
    end for;
    for piece in 1:(if degree>4 then degree else 1) loop
      if degree>4 then
        nv:=3;
        vertices[1]:=core.corners[core.first[f]+piece-1];
        vertices[2]:=core.corners[core.first[f]+mod(piece,degree)];
        vertices[3]:=midpoint[f];
      else
        nv:=degree;
        for j in 1:nv loop vertices[j]:=core.corners[core.first[f]+j-1];end for;
      end if;
      baseFace:=baseFace+1;
      mesh.first[baseFace]:=corner+1;
      for j in 1:nv loop
        corner:=corner+1;
        mesh.corners[corner]:=vertices[j];
      end for;
      mesh.patch[baseFace]:=core.patch[f];
      for side in 1:(if other>0 then 2 else 1) loop
        orig:=if side==1 then own else other;
        child:=nextCell[orig];
        if side==1 then mesh.owner[baseFace]:=child;else mesh.neighbour[baseFace]:=child;end if;
        if split[orig] then
          nextCell[orig]:=child+1;
          for j in 1:nv loop
            j2:=mod(j,nv)+1;
            u:=if side==1 then vertices[j2] else vertices[j];
            v:=if side==1 then vertices[j] else vertices[j2];
            slot:=1+mod(31*mod(31*orig+min(u,v),tableSize)+max(u,v),tableSize);
            index:=head[slot];
            while index>0 loop
              where:=baseCorners+3*(index-1);
              if parent[index]==orig and mesh.corners[where+1]==v and mesh.corners[where+2]==u then break;end if;
              index:=link[index];
            end while;
            if index==0 then
              sideCount:=sideCount+1;
              index:=sideCount;
              assert(index<=tableSize,"Inconsistent orientation at refinement transition");
              parent[index]:=orig;
              link[index]:=head[slot];
              head[slot]:=index;
              where:=baseCorners+3*(index-1);
              mesh.first[nBase+index]:=where+1;
              mesh.corners[where+1]:=u;
              mesh.corners[where+2]:=v;
              mesh.corners[where+3]:=apex[orig];
              mesh.owner[nBase+index]:=child;
            else
              mesh.neighbour[nBase+index]:=child;
            end if;
          end for;
        end if;
      end for;
    end for;
  end for;
  assert(sideCount==tableSize and min(mesh.neighbour[nBase+1:nBase+sideCount])>0,"Unclosed refinement transition");
  mesh.first[mesh.nFaces+1]:=mesh.nCorners+1;
  for cell in 1:core.nCells loop
    if split[cell] then mesh.points[apex[cell],:]:=centre[cell,:]/weight[cell];end if;
  end for;
end transitionCells;
