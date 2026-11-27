within Tests;
model MeshingCacheTests "Check volume-mesh persistence, geometry/recipe invalidation and damaged entry recovery"
  parameter String surfacePath=Modelica.Utilities.Files.loadResource("modelica://Meshing/Resources/Geometry/unitCube.stl");
  impure function check
    import F=Modelica.Utilities.Files;
    import S=Modelica.Utilities.Streams;
    input String source;
    output Boolean passed;
  protected
    String token=Meshing.Cache.temporary("mesh-cache-test"),directory=token+".d",inputPath=directory+"/cube.stl";
    String originalPath,path;
    Meshing.PolyMesh a,b;
    Boolean hit;
  algorithm
    F.createDirectory(directory);
    F.copy(source,inputPath);
    (a,hit,originalPath):=Meshing.buildSTL(inputPath,{-1,-1,-1},{3,3,3},{4,4,4},cacheDirectory=directory);
    assert(not hit,"First run must generate the mesh");
    (b,hit,path):=Meshing.buildSTL(inputPath,{-1,-1,-1},{3,3,3},{4,4,4},cacheDirectory=directory);
    assert(hit and path==originalPath and a.nCells==b.nCells and max(abs(a.points-b.points))==0 and
      max(abs(a.owner-b.owner))+max(abs(a.neighbour-b.neighbour))+max(abs(a.patch-b.patch))+
      max(abs(a.first-b.first))+max(abs(a.corners-b.corners))==0,"Incomplete mesh round trip");
    (b,hit,path):=Meshing.buildSTL(inputPath,{-1,-1,-1},{3,3,3},{6,4,4},cacheDirectory=directory);
    assert(not hit and path<>originalPath,"Grid change must invalidate the entry");
    S.print("damage",originalPath);
    S.close(originalPath);
    (b,hit,path):=Meshing.buildSTL(inputPath,{-1,-1,-1},{3,3,3},{4,4,4},cacheDirectory=directory);
    assert(not hit and path==originalPath,"Damaged mesh must be regenerated");
    S.print("",inputPath);
    S.close(inputPath);
    (b,hit,path):=Meshing.buildSTL(inputPath,{-1,-1,-1},{3,3,3},{4,4,4},cacheDirectory=directory);
    assert(not hit and path<>originalPath,"Geometry contents must contribute to the key");
    F.remove(directory);
    F.removeFile(token);
    passed:=true;
  end check;
initial algorithm
  assert(check(surfacePath),"Mesh cache regression");
end MeshingCacheTests;
