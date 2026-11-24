within Meshing;
function generate "Generate a locally refined external-flow mesh for a closed triangle surface"
  input Real lower[3](each unit="m"),upper[3](each unit="m");
  input Integer baseCells[3];
  input Integer levels(min=0);
  input Real triangles[:,3,3](each unit="m");
  input Real snapFraction(min=0,max=0.45)=0.2;
  input Real refinementBoxes[:,2,3](each unit="m")=fill(0.0,0,2,3)
    "Optional axis-aligned refinement regions as pairs of lower and upper corners";
  input Integer boxLevels[size(refinementBoxes,1)]=fill(levels,size(refinementBoxes,1))
    "Target refinement level inside each region";
  input Integer smoothSurfaceLevel(min=0)=levels "Refinement on faces away from sharp edges";
  input Real featureAngle(unit="rad")=Modelica.Constants.pi/4 "Dihedral threshold for finest surface refinement";
  output PolyMesh mesh;
protected
  Integer n[3]=baseCells*integer(2^levels);
  Integer ids[n[1],n[2],n[3]],stride[n[1],n[2],n[3]],nCells;
algorithm
  (ids,stride,nCells):=classifyAndRefine(lower,upper,baseCells,levels,triangles,refinementBoxes,boxLevels,smoothSurfaceLevel,featureAngle);
  assert(size(triangles,1)==0 or min(ids)<0,
    "No solid cell was resolved; refine the mesh or check the geometry units");
  mesh:=buildTopology(ids,stride,nCells,lower,upper);
  if snapFraction>0 and size(triangles,1)>0 then
    mesh.points:=snapWall(mesh,triangles,lower,(upper-lower)./n,n,snapFraction);
  end if;
end generate;
