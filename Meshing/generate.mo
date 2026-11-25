within Meshing;
function generate "Generate a locally refined external-flow mesh for a closed triangle surface"
  input Real lower[3](each unit="m"),upper[3](each unit="m");
  input Integer baseCells[3];
  input Integer levels(min=0);
  input Real triangles[:,3,3](each unit="m");
  input Real snapFraction(min=0,max=0.6)=0.2;
  input Real refinementBoxes[:,2,3](each unit="m")=fill(0.0,0,2,3)
    "Optional axis-aligned refinement regions as pairs of lower and upper corners";
  input Integer boxLevels[size(refinementBoxes,1)]=fill(levels,size(refinementBoxes,1))
    "Target refinement level inside each region";
  input Integer smoothSurfaceLevel(min=0)=levels "Refinement on faces away from sharp edges";
  input Real featureAngle(unit="rad")=Modelica.Constants.pi/4 "Dihedral threshold for finest surface refinement";
  input Boolean captureThinSolids=false "Capture intersected cells where adjacent centres miss the solid";
  input Method method=if captureThinSolids then Method.thinWallCartesian else Method.snappedCartesian;
  output PolyMesh mesh;
protected
  Real phi[:,:,:];
  Integer n[3]=baseCells*integer(2^levels);
  Integer ids[n[1],n[2],n[3]],stride[n[1],n[2],n[3]],nCells;
algorithm
  if method==Method.levelSetCut then
    assert(size(refinementBoxes,1)==0 and smoothSurfaceLevel==levels,
      "The level-set method uses uniformly subdivided base cells; use levelSetMesh for prescribed axes");
    phi:=surfaceDistanceGrid(triangles,lower,upper,n);
    assert(size(triangles,1)==0 or min(phi)<0,"No solid interior node was resolved. Refine the level-set grid.");
    mesh:=levelSetMesh(linspace(lower[1],upper[1],n[1]+1),linspace(lower[2],upper[2],n[2]+1),
      linspace(lower[3],upper[3],n[3]+1),phi);
    return;
  end if;
  (ids,stride,nCells):=classifyAndRefine(lower,upper,baseCells,levels,triangles,refinementBoxes,boxLevels,smoothSurfaceLevel,featureAngle,method==Method.thinWallCartesian);
  assert(size(triangles,1)==0 or min(ids)<0,
    "No solid cell was resolved; refine the mesh or check the geometry units");
  mesh:=buildTopology(ids,stride,nCells,lower,upper);
  if snapFraction>0 and size(triangles,1)>0 then
    mesh.points:=snapWall(mesh,triangles,lower,(upper-lower)./n,n,snapFraction);
  end if;
  mesh:=transitionCells(mesh);
end generate;
