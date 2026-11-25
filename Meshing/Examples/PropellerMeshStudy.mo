within Meshing.Examples;
model PropellerMeshStudy "Compare native volume meshes of the W2025 port propeller (metres)"
  parameter String surfacePath="port.stl" "Closed, shaft-aligned CAD-derived port propeller";
  parameter Meshing.Method method=Meshing.Method.thinWallCartesian;
  parameter Integer grid[3]={30,48,48} "Finest grid cell counts, each divisible by two";
  parameter String meshDirectory="" "Optional polyMesh export directory";
  parameter String wallPath="" "Optional outward-wound STL for rasterization";
  final parameter Integer cells(fixed=false),shells(fixed=false);
  final parameter Real referenceArea(fixed=false),wallArea(fixed=false),referenceVolume(fixed=false),
    wallVolume(fixed=false),minimumCellVolume(fixed=false);
protected
  impure function measure
    input String path,directory,wallPath;
    input Meshing.Method method;
    input Integer grid[3];
    output Integer cells,shells;
    output Real referenceArea,wallArea,referenceVolume,wallVolume,minimumVolume;
  protected
    Real triangles[:,3,3],volumes[:];
    Meshing.PolyMesh m;
    Integer base[3];
  algorithm
    triangles:=Meshing.readAsciiSTL(path);
    for d in 1:3 loop
      assert(grid[d]>0 and mod(grid[d],2)==0,"Positive even grid counts required");
      base[d]:=div(grid[d],2);
    end for;
    m:=Meshing.generate({-0.026,-0.064,-0.064},{0.026,0.064,0.064},base,1,triangles,0.6,method=method);
    m.points:=Meshing.stretchFarField(m,{-0.018,-0.055,-0.055},{0.018,0.055,0.055},
      {-0.3198,-0.2132,-0.2132},{0.6396,0.2132,0.2132});
    volumes:=Meshing.cellVolumes(m,true);
    minimumVolume:=min(volumes);
    cells:=m.nCells;shells:=Meshing.bodyComponentCount(m);
    (referenceArea,wallArea,referenceVolume,wallVolume):=Meshing.bodySurfaceMetrics(m,triangles);
    Modelica.Utilities.Streams.print("mesh study: cells="+String(cells)+", shells="+String(shells)+
      ", wallArea="+String(wallArea,significantDigits=12)+", referenceArea="+String(referenceArea,significantDigits=12)+
      ", wallVolume="+String(wallVolume,significantDigits=12)+", referenceVolume="+String(referenceVolume,significantDigits=12)+
      ", minimumVolume="+String(minimumVolume,significantDigits=12));
    if wallPath<>"" then Meshing.writeBodySTL(wallPath,m); end if;
    if directory<>"" then Meshing.writePolyMesh(directory,m);end if;
  end measure;
initial algorithm
  (cells,shells,referenceArea,wallArea,referenceVolume,wallVolume,minimumCellVolume):=
    measure(surfacePath,meshDirectory,wallPath,method,grid);
end PropellerMeshStudy;
