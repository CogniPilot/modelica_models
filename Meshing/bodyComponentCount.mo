within Meshing;
function bodyComponentCount "Count vertex-connected shells on the generated solid wall patch"
  input PolyMesh mesh;
  output Integer count;
protected
  Integer parent[mesh.nPoints]=fill(0,mesh.nPoints);
  Integer a,b,v;
algorithm
  for f in 1:mesh.nFaces loop
    if mesh.patch[f]==7 then
      a:=mesh.corners[mesh.first[f]];
      if parent[a]==0 then parent[a]:=a;end if;
      while parent[a]<>a loop
        parent[a]:=parent[parent[a]];
        a:=parent[a];
      end while;
      for c in mesh.first[f]+1:mesh.first[f+1]-1 loop
        v:=mesh.corners[c];
        if parent[v]==0 then
          parent[v]:=a;
        else
          b:=v;
          while parent[b]<>b loop
            parent[b]:=parent[parent[b]];
            b:=parent[b];
          end while;
          parent[b]:=a;
        end if;
      end for;
    end if;
  end for;
  count:=sum(if parent[i]==i then 1 else 0 for i in 1:mesh.nPoints);
end bodyComponentCount;
