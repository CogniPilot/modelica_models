within Meshing;
function buildTopology "Join adjacent leaves with shared, outward-wound polyhedral faces"
  input Integer cellId[:,:,:];
  input Integer stride[size(cellId,1),size(cellId,2),size(cellId,3)];
  input Integer nCells;
  input Real lower[3](each unit="m"),upper[3](each unit="m");
  output PolyMesh mesh;
protected
  Integer n[3]=size(cellId);
  Integer pointId[product(n+{1,1,1})]=fill(0,product(n+{1,1,1}));
  Real h[3]=(upper-lower)./n;
  Integer own[:],other[:],boundary[:],first[:],corners[:];
  Real points[:,3];
  Integer faces=0,vertices=0,count=0;
  Integer u,v,minus,plus,s,sm,sp,label,idx[3],pt[3],id,pos;
  Boolean positive;
algorithm
  // Mark leaf corners, count incident face vertices, then fill exact-sized arrays. Integer coordinates identify
  // shared vertices even where a large face meets several smaller leaves.
  for pass in 1:3 loop
    if pass==3 then
      own:=fill(0,faces);
      other:=fill(0,faces);
      boundary:=fill(0,faces);
      first:=fill(0,faces+1);
      corners:=fill(0,count);
      points:=zeros(vertices,3);
    end if;
    faces:=0;
    count:=0;
    for axis in 1:3 loop
      u:=mod(axis,3)+1;
      v:=mod(axis+1,3)+1;
      for plane in 0:n[axis] loop
        for b in 0:n[v]-1 loop
          for a in 0:n[u]-1 loop
            idx[u]:=a+1;
            idx[v]:=b+1;
            idx[axis]:=max(1,plane);
            minus:=if plane==0 then
              0 else max(0,cellId[idx[1],idx[2],idx[3]]);
            sm:=if minus>0 then
              stride[idx[1],idx[2],idx[3]] else 0;
            idx[axis]:=min(n[axis],plane+1);
            plus:=if plane==n[axis] then
              0 else max(0,cellId[idx[1],idx[2],idx[3]]);
            sp:=if plus>0 then
              stride[idx[1],idx[2],idx[3]] else 0;
            if plus<>minus then
              s:=if min(sm,sp)>0 then
                min(sm,sp) else max(sm,sp);
              if mod(a,s)==0 and mod(b,s)==0 then
                positive:=minus>0 and (plus==0 or minus<plus);
                label:=if min(minus,plus)>0 then
                  0 else if plane==0 then
                    2*axis-1 else if plane==n[axis] then
                      2*axis else 7;
                faces:=faces+1;
                if pass==3 then
                  own[faces]:=if positive then
                    minus else plus;
                  other[faces]:=if positive then
                    plus else minus;
                  boundary[faces]:=label;
                  first[faces]:=count+1;
                end if;
                // Subdivision points on each perimeter keep all edge loops
                // conforming at refinement transitions.
                for j in 0:4*s-1 loop
                  pos:=if positive then
                    j else mod(4*s-j,4*s);
                  pt[axis]:=plane;
                  pt[u]:=a;
                  pt[v]:=b;
                  if pos<s then
                    pt[u]:=a+pos;
                  elseif pos<2*s then
                    pt[u]:=a+s;
                    pt[v]:=b+pos-s;
                  elseif pos<3*s then
                    pt[u]:=a+3*s-pos;
                    pt[v]:=b+s;
                  else
                    pt[v]:=b+4*s-pos;
                  end if;
                  id:=pointId[1+pt[1]+(n[1]+1)*(pt[2]+(n[2]+1)*pt[3])];
                  if pass==1 and mod(pos,s)==0 then
                    pointId[1+pt[1]+(n[1]+1)*(pt[2]+(n[2]+1)*pt[3])]:=-1;
                  elseif pass==2 and id<0 then
                    vertices:=vertices+1;
                    id:=vertices;
                    pointId[1+pt[1]+(n[1]+1)*(pt[2]+(n[2]+1)*pt[3])]:=id;
                  end if;
                  if id<>0 then
                    count:=count+1;
                  end if;
                  if pass==3 and id>0 then
                    points[id,:]:=lower+pt.*h;
                    corners[count]:=id;
                  end if;
                end for;
              end if;
            end if;
          end for;
        end for;
      end for;
    end for;
  end for;
  first[faces+1]:=count+1;
  mesh:=PolyMesh(vertices,faces,count,nCells,points,first,corners,own,other,boundary);
end buildTopology;
