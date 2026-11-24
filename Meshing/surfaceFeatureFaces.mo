within Meshing;
function surfaceFeatureFaces "Validate edge incidence and identify triangles bordering a crease"
  input Real triangles[:,3,3](each unit="m") "Closed surface with shared vertex coordinates";
  input Real featureAngle(unit="rad")=Modelica.Constants.pi/4;
  output Boolean feature[size(triangles,1)];
protected
  Integer count=size(triangles,1),capacity=max(1,3*count);
  Integer head[capacity]=fill(0,capacity),link[capacity],firstFace[capacity],firstSide[capacity];
  Boolean paired[capacity]=fill(false,capacity);
  Real normals[count,3],origin[3],extent[3],a[3],b[3],c[3],u[3],v[3],scale[3],cosine;
  Integer key,edge,total=0,previous,side,nextSide,bin[3];
  Boolean same,reverse;
algorithm
  assert(featureAngle>=0 and featureAngle<=Modelica.Constants.pi,"Feature angle must lie between zero and pi");
  feature:=fill(false,count);
  if count==0 then return;end if;
  for d in 1:3 loop
    origin[d]:=min(triangles[:,:,d]);
    extent[d]:=max(triangles[:,:,d])-origin[d];
  end for;
  assert(min(extent)>0,"Closed solid needs three-dimensional extent");
  scale:=fill(4096.0,3)./extent;
  cosine:=cos(featureAngle);
  for f in 1:count loop
    a:=triangles[f,1,:];
    b:=triangles[f,2,:];
    c:=triangles[f,3,:];
    u:=cross(b-a,c-a);
    assert(u*u>0,"Degenerate surface triangle "+String(f));
    normals[f,:]:=u/sqrt(u*u);
    for j in 1:3 loop
      a:=triangles[f,j,:];
      b:=triangles[f,mod(j,3)+1,:];
      // Bins accelerate lookup only. The endpoint comparison decides identity.
      for d in 1:3 loop
        bin[d]:=integer(floor(((a[d]-origin[d])+(b[d]-origin[d]))*0.5*scale[d]));
      end for;
      key:=1+mod(bin[1]*65537+bin[2]*257+bin[3],capacity);
      edge:=head[key];
      while edge>0 loop
        previous:=firstFace[edge];
        side:=firstSide[edge];
        nextSide:=mod(side,3)+1;
        u:=triangles[previous,side,:];
        v:=triangles[previous,nextSide,:];
        same:=max(abs(a-u))==0 and max(abs(b-v))==0;
        reverse:=max(abs(a-v))==0 and max(abs(b-u))==0;
        if same or reverse then break;end if;
        edge:=link[edge];
      end while;
      if edge==0 then
        total:=total+1;
        firstFace[total]:=f;
        firstSide[total]:=j;
        link[total]:=head[key];
        head[key]:=total;
      else
        assert(not paired[edge],"Nonmanifold edge at triangle "+String(f));
        paired[edge]:=true;
        // If the supplied winding flips at this edge, compare corrected normals.
        if (if same then -1 else 1)*(normals[f,:]*normals[previous,:])<cosine then
          feature[f]:=true;
          feature[previous]:=true;
        end if;
      end if;
    end for;
  end for;
  for e in 1:total loop
    assert(paired[e],"Open edge at triangle "+String(firstFace[e])+"; check watertightness and coordinate units");
  end for;
end surfaceFeatureFaces;
