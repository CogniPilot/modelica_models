within Meshing;
impure function buildSTL "Build or reuse a validated exterior volume mesh"
  input String surfacePath;
  input Real lower[3](each unit="m"),upper[3](each unit="m");
  input Integer baseCells[3],levels(min=0)=1;
  input Real metresPerUnit=1, snapFraction(min=0,max=0.6)=0.2;
  input Integer layers(min=0)=0;
  input Real layerThickness(unit="m")=0 "Total layer thickness measured into the fluid";
  input Real growth(min=1)=1.2;
  input Real refinementBoxes[:,2,3](each unit="m")=fill(0.0,0,2,3)
    "Optional axis-aligned refinement regions as pairs of lower and upper corners";
  input Integer boxLevels[size(refinementBoxes,1)]=fill(levels,size(refinementBoxes,1))
    "Target refinement level inside each region";
  input Integer smoothSurfaceLevel(min=0)=levels "Refinement on faces away from sharp edges";
  input Real featureAngle(unit="rad")=Modelica.Constants.pi/4 "Dihedral threshold for finest surface refinement";
  input Boolean captureThinSolids=false "Capture intersected cells where adjacent centres miss the solid";
  input Integer expectedBodyComponents(min=0)=0 "Expected number of wall shells; zero skips this check";
  input Method method=if captureThinSolids then Method.thinWallCartesian else Method.snappedCartesian;
  input String cacheDirectory=".meshing-cache" "Empty disables caching";
  output PolyMesh mesh;
  output Boolean cacheHit;
  output String artifactPath;
protected
  String recipe=Cache.generatorVersion+" "+String(method)+" "+String(levels)+" "+String(smoothSurfaceLevel)+" "+String(layers);
  String digest;
  Real triangles[:,3,3],volumes[:];
algorithm
  cacheHit:=false;
  artifactPath:="";
  if cacheDirectory<>"" then
    for value in {metresPerUnit,snapFraction,layerThickness,growth,featureAngle} loop
      recipe:=recipe+" "+String(value,significantDigits=17);
    end for;
    for d in 1:3 loop
      recipe:=recipe+" "+String(lower[d],significantDigits=17)+" "+String(upper[d],significantDigits=17)+" "+String(baseCells[d]);
    end for;
    for b in 1:size(refinementBoxes,1) loop
      recipe:=recipe+" box "+String(boxLevels[b]);
      for corner in 1:2 loop
        for d in 1:3 loop recipe:=recipe+" "+String(refinementBoxes[b,corner,d],significantDigits=17);end for;
      end for;
    end for;
    digest:=Cache.key(surfacePath,recipe);
    artifactPath:=cacheDirectory+"/"+digest+".mat";
    cacheHit:=Cache.verified(artifactPath,artifactPath+".sha256",digest);
  end if;
  if cacheHit then
    mesh:=loadMesh(artifactPath);
  else
    triangles:=readAsciiSTL(surfacePath,metresPerUnit);
    mesh:=generate(lower,upper,baseCells,levels,triangles,snapFraction,refinementBoxes,boxLevels,
      smoothSurfaceLevel,featureAngle,captureThinSolids,method);
    if layers>0 then mesh:=addWallLayers(mesh,layers,layerThickness,growth);end if;
    volumes:=cellVolumes(mesh,checkFacePyramids=true);
    assert(min(volumes)>0,"Mesh quality check failed");
  end if;
  if expectedBodyComponents>0 then
    assert(bodyComponentCount(mesh)==expectedBodyComponents,"Unexpected number of connected solid wall components");
  end if;
  if cacheDirectory<>"" and not cacheHit then
    assert(Cache.key(surfacePath,recipe)==digest,"Source geometry changed while generating mesh");
    Modelica.Utilities.Files.createDirectory(cacheDirectory);
    saveMesh(artifactPath,mesh);
    Cache.sign(artifactPath,artifactPath+".sha256",digest);
  end if;
end buildSTL;
