within Meshing;
function stretchFarField "Expand an axis-aligned outer mesh while keeping near-body coordinates fixed"
  input PolyMesh mesh;
  input Real fixedLower[3](each unit="m"),fixedUpper[3](each unit="m")
    "Bounds of the region kept at its original physical resolution";
  input Real outerLower[3](each unit="m"),outerUpper[3](each unit="m");
  output Real points[mesh.nPoints,3](each unit="m");
protected
  Real lower,upper,x,s;
algorithm
  points:=mesh.points;
  for d in 1:3 loop
    lower:=min(mesh.points[:,d]);
    upper:=max(mesh.points[:,d]);
    assert(outerLower[d]<=lower and lower<fixedLower[d] and fixedLower[d]<fixedUpper[d]
      and fixedUpper[d]<upper and upper<=outerUpper[d],"Invalid near-field or outer bounds");
    for i in 1:mesh.nPoints loop
      x:=mesh.points[i,d];
      if x<fixedLower[d] then
        s:=(fixedLower[d]-x)/(fixedLower[d]-lower);
        points[i,d]:=x+(outerLower[d]-lower)*s^4;
      elseif x>fixedUpper[d] then
        s:=(x-fixedUpper[d])/(upper-fixedUpper[d]);
        points[i,d]:=x+(outerUpper[d]-upper)*s^4;
      end if;
    end for;
  end for;
end stretchFarField;
