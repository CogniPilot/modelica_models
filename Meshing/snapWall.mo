within Meshing;
function snapWall "Project castellated wall vertices toward the closest point on the triangle surface"
  input PolyMesh mesh;
  input Real triangles[:,3,3](each unit="m"), lower[3](each unit="m"), spacing[3](each unit="m");
  input Integer n[3] "Finest background resolution";
  input Real fraction(min=0,max=0.45) "Maximum displacement in units of finest cell width";
  output Real points[mesh.nPoints,3](each unit="m");
protected
  Integer wallPoint[n[1]+1,n[2]+1,n[3]+1]=fill(0,n[1]+1,n[2]+1,n[3]+1);
  Real best[mesh.nPoints]=fill(Modelica.Constants.inf,mesh.nPoints);
  Real p[3],a[3],b[3],c[3],q[3],normal[3],edge[3],start[3],candidate[3],d[3],dd,t;
  Real radius=2*sqrt(spacing*spacing);
  Integer index[3],lo[3],hi[3],id;
algorithm
  assert(fraction>=0 and fraction<=0.45,"Snap fraction must be between 0 and 0.45");
  points:=mesh.points;
  for f in 1:mesh.nFaces loop
    if mesh.patch[f]==7 then
      for j in mesh.first[f]:mesh.first[f+1]-1 loop
        id:=mesh.corners[j];
        for k in 1:3 loop
          index[k]:=1+integer(floor(0.5+(points[id,k]-lower[k])/spacing[k]));
        end for;
        wallPoint[index[1],index[2],index[3]]:=id;
      end for;
    end if;
  end for;
  for face in 1:size(triangles,1) loop
    a:=triangles[face,1,:];
    b:=triangles[face,2,:];
    c:=triangles[face,3,:];
    normal:=cross(b-a,c-a);
    for j in 1:3 loop
      lo[j]:=max(0,integer(ceil((min(triangles[face,:,j])-radius-lower[j])/spacing[j])));
      hi[j]:=min(n[j],integer(floor((max(triangles[face,:,j])+radius-lower[j])/spacing[j])));
    end for;
    for k in lo[3]:hi[3] loop
      for j in lo[2]:hi[2] loop
        for i in lo[1]:hi[1] loop
          id:=wallPoint[i+1,j+1,k+1];
          if id>0 and normal*normal>0 then
            p:=mesh.points[id,:];
            q:=p-normal*((p-a)*normal)/(normal*normal);
            if cross(b-a,q-a)*normal<0 or cross(c-b,q-b)*normal<0 or cross(a-c,q-c)*normal<0 then
              dd:=Modelica.Constants.inf;
              for e in 1:3 loop
                start:=triangles[face,e,:];
                edge:=triangles[face,mod(e,3)+1,:]-start;
                t:=max(0,min(1,(p-start)*edge/(edge*edge)));
                candidate:=start+t*edge;
                d:=candidate-p;
                if d*d<dd then
                  q:=candidate;
                  dd:=d*d;
                end if;
              end for;
            end if;
            d:=q-p;
            dd:=d*d;
            if dd<best[id] then
              best[id]:=dd;
              points[id,:]:=p+d*min(1,fraction*min(spacing)/max(sqrt(dd),1e-30));
            end if;
          end if;
        end for;
      end for;
    end for;
  end for;
  points:=limitHexSnap(mesh,points);
end snapWall;
