within Meshing;
function classifyAndRefine "Tag exterior fluid leaves on a locally refined Cartesian background"
  input Real lower[3](each unit="m");
  input Real upper[3](each unit="m");
  input Integer baseCells[3];
  input Integer levels(min=0);
  input Real triangles[:,3,3](each unit="m") "Closed watertight triangle soup";
  input Real refinementBoxes[:,2,3](each unit="m")=fill(0.0,0,2,3)
    "Optional axis-aligned refinement regions as pairs of lower and upper corners";
  input Integer boxLevels[size(refinementBoxes,1)]=fill(levels,size(refinementBoxes,1))
    "Target refinement level inside each region";
  input Integer smoothSurfaceLevel(min=0)=levels "Refinement on faces away from sharp edges";
  input Real featureAngle(unit="rad")=Modelica.Constants.pi/4 "Dihedral threshold for finest surface refinement";
  input Boolean captureThinSolids=false "Capture intersected cells where adjacent centres miss the solid";
  output Integer cellId[baseCells[1]*integer(2^levels),baseCells[2]*integer(2^levels),baseCells[3]*integer(2^levels)];
  output Integer stride[size(cellId,1),size(cellId,2),size(cellId,3)] "Finest-grid intervals per leaf edge";
  output Integer nCells;
protected
  Boolean feature[size(triangles,1)];
  Integer n[3]=size(cellId);
  Real h[3]=(upper-lower)./n;
  Integer required[n[1],n[2],n[3]]=fill(0,n[1],n[2],n[3]);
  Integer crossings[n[1]+1,n[2],n[3]]=fill(0,n[1]+1,n[2],n[3]);
  Integer lo[3],hi[3],first,parity,s;
  Real a[3],b[3],c[3],det,p[2],u,v,hit;
algorithm
  assert(min(baseCells)>0 and levels>=0 and min(upper-lower)>0,"Invalid background domain");
  assert(smoothSurfaceLevel>=0 and smoothSurfaceLevel<=levels,"Invalid smooth surface refinement level");
  feature:=surfaceFeatureFaces(triangles,featureAngle);
  cellId:=fill(0,n[1],n[2],n[3]);
  stride:=fill(0,n[1],n[2],n[3]);
  for region in 1:size(refinementBoxes,1) loop
    assert(boxLevels[region]>=0 and boxLevels[region]<=levels and
      min(refinementBoxes[region,2,:]-refinementBoxes[region,1,:])>0,"Invalid refinement region");
    for level in 1:boxLevels[region] loop
      s:=integer(2^(levels-level));
      for d in 1:3 loop
        lo[d]:=max(1,1+integer(floor((refinementBoxes[region,1,d]-lower[d])/h[d]))-s);
        hi[d]:=min(n[d],integer(ceil((refinementBoxes[region,2,d]-lower[d])/h[d]))+s);
      end for;
      for k in lo[3]:hi[3] loop
        for j in lo[2]:hi[2] loop
          for i in lo[1]:hi[1] loop
            required[i,j,k]:=max(required[i,j,k],level);
          end for;
        end for;
      end for;
    end for;
  end for;
  for t in 1:size(triangles,1) loop
    a:=triangles[t,1,:];
    b:=triangles[t,2,:];
    c:=triangles[t,3,:];
    for d in 1:3 loop
      assert(min(triangles[t,:,d])>lower[d] and max(triangles[t,:,d])<upper[d],
        "Body surface must lie strictly inside the background domain");
    end for;
    assert(cross(b-a,c-a)*cross(b-a,c-a)>0,"Degenerate triangle in body surface");
    // Nested padding places a band of coarser leaves around each surface.
    for level in 1:(if feature[t] then levels else smoothSurfaceLevel) loop
      s:=integer(2^(levels-level));
      for d in 1:3 loop
        lo[d]:=max(1,integer(floor((min(triangles[t,:,d])-lower[d])/h[d]))+1-2*s);
        hi[d]:=min(n[d],integer(floor((max(triangles[t,:,d])-lower[d])/h[d]))+1+2*s);
      end for;
      for k in lo[3]:hi[3] loop
        for j in lo[2]:hi[2] loop
          for i in lo[1]:hi[1] loop
            required[i,j,k]:=max(required[i,j,k],level);
          end for;
        end for;
      end for;
    end for;
    // Intersect parallel scanlines with projected triangles. A common sub-cell
    // offset breaks ties on regular triangulations without moving mesh nodes.
    det:=(b[2]-a[2])*(c[3]-a[3])-(b[3]-a[3])*(c[2]-a[2]);
    if abs(det)>1e-14*h[2]*h[3] then
      for d in 2:3 loop
        lo[d]:=max(1,integer(floor((min(triangles[t,:,d])-lower[d])/h[d])));
        hi[d]:=min(n[d],integer(ceil((max(triangles[t,:,d])-lower[d])/h[d]))+1);
      end for;
      for k in lo[3]:hi[3] loop
        for j in lo[2]:hi[2] loop
          p:={lower[2]+(j-0.5+1.32e-8)*h[2]-a[2],lower[3]+(k-0.5+2.71e-8)*h[3]-a[3]};
          u:=(p[1]*(c[3]-a[3])-p[2]*(c[2]-a[2]))/det;
          v:=((b[2]-a[2])*p[2]-(b[3]-a[3])*p[1])/det;
          if u>=0 and v>=0 and u+v<1 then
            hit:=a[1]+u*(b[1]-a[1])+v*(c[1]-a[1]);
            first:=max(1,min(n[1]+1,integer(floor((hit-lower[1])/h[1]+0.5))+1));
            crossings[first,j,k]:=crossings[first,j,k]+1;
          end if;
        end for;
      end for;
    end if;
  end for;
  for k in 1:n[3] loop
    for j in 1:n[2] loop
      parity:=0;
      for i in 1:n[1] loop
        parity:=mod(parity+crossings[i,j,k],2);
        cellId[i,j,k]:=-parity;
      end for;
      assert(mod(parity+crossings[n[1]+1,j,k],2)==0,"Surface is open or has an ambiguous crossing");
    end for;
  end for;
  if size(triangles,1)>0 then
    if captureThinSolids then
      cellId:=markSurfaceCells(cellId,triangles,lower,h);
    end if;
    cellId:=regularizeSurface(cellId);
  end if;
  nCells:=0;
  for level in 0:levels loop
    s:=integer(2^(levels-level));
    for k in 1:s:n[3] loop
      for j in 1:s:n[2] loop
        for i in 1:s:n[1] loop
          if cellId[i,j,k]==0 and max(required[i:i+s-1,j:j+s-1,k:k+s-1])<=level
              and min(cellId[i:i+s-1,j:j+s-1,k:k+s-1])==0 then
            nCells:=nCells+1;
            cellId[i:i+s-1,j:j+s-1,k:k+s-1]:=fill(nCells,s,s,s);
            stride[i:i+s-1,j:j+s-1,k:k+s-1]:=fill(s,s,s,s);
          end if;
        end for;
      end for;
    end for;
  end for;
  assert(nCells>0,"No exterior fluid cells remain; check domain and surface units");
end classifyAndRefine;
