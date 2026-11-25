within Meshing.Examples;
model SampledTetStudy "Clip an unstructured nodal field and measure its exterior volume mesh"
  parameter String fieldPath="field.mat" "Real MAT matrices points[N,3], tets[K,4], patch[K,4], phi[N,1]";
  parameter String surfacePath="port.stl" "Reference surface in metres";
  parameter String wallPath="" "Optional outward-wound STL export";
  parameter Integer layers(min=0)=0;
  parameter Real layerThickness(unit="m")=0;
  parameter String cacheDirectory="" "Save and reuse the full volume mesh";
  final parameter String artifactPath(fixed=false);
  final parameter Boolean cacheHit(fixed=false);
  final parameter Integer cells(fixed=false),shells(fixed=false);
  final parameter Real referenceArea(fixed=false),wallArea(fixed=false),referenceVolume(fixed=false),
    wallVolume(fixed=false),minimumCellVolume(fixed=false),fluidVolume(fixed=false);
protected
  impure function build
    import Streams=Modelica.Utilities.Streams;
    input String fieldPath;
    input Integer layers;
    input Real layerThickness;
    output Meshing.PolyMesh m;
  protected
    Integer nodes[2]=Streams.readMatrixSize(fieldPath,"points"),elements[2]=Streams.readMatrixSize(fieldPath,"tets");
    Real p[nodes[1],3]=Streams.readRealMatrix(fieldPath,"points",nodes[1],3);
    Real t[elements[1],4]=Streams.readRealMatrix(fieldPath,"tets",elements[1],4);
    Real ids[elements[1],4]=Streams.readRealMatrix(fieldPath,"patch",elements[1],4);
    Real field[nodes[1],1]=Streams.readRealMatrix(fieldPath,"phi",nodes[1],1);
    Integer tetrahedra[elements[1],4],patch[elements[1],4];
  algorithm
    for i in 1:elements[1] loop
      for j in 1:4 loop
        tetrahedra[i,j]:=integer(t[i,j]);
        patch[i,j]:=integer(ids[i,j]);
        assert(tetrahedra[i,j]==t[i,j] and patch[i,j]==ids[i,j],"Indices and patch IDs must be integral");
      end for;
    end for;
    m:=Meshing.clipTetrahedra(p,tetrahedra,field[:,1],patch);
    if layers>0 then
      m:=Meshing.addWallLayers(m,layers,layerThickness);
    end if;
  end build;
  impure function measure
    import Streams=Modelica.Utilities.Streams;
    input String fieldPath,surfacePath,wallPath,cacheDirectory;
    input Integer layers;
    input Real layerThickness;
    output Integer cells,shells;
    output Real referenceArea,wallArea,referenceVolume,wallVolume,minimumVolume,fluidVolume;
    output String artifactPath;
    output Boolean cacheHit;
  protected
    String digest,recipe="Meshing.SampledTetStudy/1 "+String(layers)+" "+String(layerThickness,significantDigits=17);
    Meshing.PolyMesh m;
    Real triangles[:,3,3],volumes[:];
  algorithm
    artifactPath:="";
    cacheHit:=false;
    if cacheDirectory<>"" then
      digest:=Cache.key(fieldPath,recipe);
      artifactPath:=cacheDirectory+"/"+digest+".mat";
      cacheHit:=Cache.verified(artifactPath,artifactPath+".sha256",digest);
    end if;
    if cacheHit then m:=Meshing.loadMesh(artifactPath);
    else m:=build(fieldPath,layers,layerThickness);end if;
    volumes:=Meshing.cellVolumes(m,true);
    cells:=m.nCells;
    shells:=Meshing.bodyComponentCount(m);
    minimumVolume:=min(volumes);
    fluidVolume:=sum(volumes);
    triangles:=Meshing.readAsciiSTL(surfacePath);
    (referenceArea,wallArea,referenceVolume,wallVolume):=Meshing.bodySurfaceMetrics(m,triangles);
    if cacheDirectory<>"" and not cacheHit then
      assert(Cache.key(fieldPath,recipe)==digest,"The input field changed during generation");
      Modelica.Utilities.Files.createDirectory(cacheDirectory);
      Meshing.saveMesh(artifactPath,m);
      Cache.sign(artifactPath,artifactPath+".sha256",digest);
    end if;
    if wallPath<>"" then Meshing.writeBodySTL(wallPath,m); end if;
  end measure;
initial algorithm
  (cells,shells,referenceArea,wallArea,referenceVolume,wallVolume,minimumCellVolume,fluidVolume,artifactPath,cacheHit):=
    measure(fieldPath,surfacePath,wallPath,cacheDirectory,layers,layerThickness);
end SampledTetStudy;
