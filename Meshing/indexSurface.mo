within Meshing;
function indexSurface "Index a triangle soup by exactly shared vertex coordinates"
  input Real triangles[:,3,3](each unit="m");
  output Real points[:,3](each unit="m");
  output Integer faces[size(triangles,1),3];
protected
  Integer capacity=max(1,3*size(triangles,1)),count=0;
  Real vertices[capacity,3],origin[3],width[3],p[3];
  Integer bins[3],bucket,id,head[capacity]=fill(0,capacity),link[capacity];
algorithm
  assert(size(triangles,1)>0,"A nonempty surface is required");
  for d in 1:3 loop
    origin[d]:=min(triangles[:,:,d]);width[d]:=max(triangles[:,:,d])-origin[d];
  end for;
  for f in 1:size(triangles,1) loop
    for c in 1:3 loop
      p:=triangles[f,c,:];
      for d in 1:3 loop
        bins[d]:=if width[d]>0 then integer(floor(4096*(p[d]-origin[d])/width[d])) else 0;
      end for;
      bucket:=1+mod(bins[1]*65537+bins[2]*257+bins[3],capacity);
      id:=head[bucket];
      while id>0 loop
        if max(abs(vertices[id,:]-p))==0 then break;end if;
        id:=link[id];
      end while;
      if id==0 then
        count:=count+1;id:=count;vertices[id,:]:=p;link[id]:=head[bucket];head[bucket]:=id;
      end if;
      faces[f,c]:=id;
    end for;
  end for;
  points:=vertices[1:count,:];
end indexSurface;
