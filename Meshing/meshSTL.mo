within Meshing;
impure function meshSTL "Generate and verify an exterior volume mesh, then export OpenFOAM mesh objects"
  input String surfacePath, meshDirectory;
  input Real lower[3](each unit="m"),upper[3](each unit="m");
  input Integer baseCells[3],levels(min=0)=1;
  input Real metresPerUnit=1, snapFraction(min=0,max=0.45)=0.2;
  input Integer layers(min=0)=0;
  input Real layerThickness(unit="m")=0 "Total layer thickness measured into the fluid";
  input Real growth(min=1)=1.2;
  input Real refinementBoxes[:,2,3](each unit="m")=fill(0.0,0,2,3)
    "Optional axis-aligned refinement regions as pairs of lower and upper corners";
  input Integer boxLevels[size(refinementBoxes,1)]=fill(levels,size(refinementBoxes,1))
    "Target refinement level inside each region";
  input Integer smoothSurfaceLevel(min=0)=levels "Refinement on faces away from sharp edges";
  input Real featureAngle(unit="rad")=Modelica.Constants.pi/4 "Dihedral threshold for finest surface refinement";
  output Integer cells;
protected
  Real triangles[:,3,3],volumes[:];
  PolyMesh mesh;
algorithm
  triangles:=readAsciiSTL(surfacePath,metresPerUnit);
  mesh:=generate(lower,upper,baseCells,levels,triangles,snapFraction,refinementBoxes,boxLevels,smoothSurfaceLevel,featureAngle);
  if layers>0 then
    mesh:=addWallLayers(mesh,layers,layerThickness,growth);
  end if;
  volumes:=cellVolumes(mesh);
  assert(min(volumes)>0,"Mesh quality check failed");
  writePolyMesh(meshDirectory,mesh);
  cells:=mesh.nCells;
end meshSTL;
