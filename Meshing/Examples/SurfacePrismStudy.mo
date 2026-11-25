within Meshing.Examples;
model SurfacePrismStudy "Measure a fitted prism block on a closed, outward-wound STL in metres"
  parameter String surfacePath="port.stl", wallPath="";
  parameter Real thickness(unit="m")=0.00005 "Requested total thickness across three layers";
  parameter Boolean limitThickness=true;
  final parameter Integer cells(fixed=false),shells(fixed=false);
  final parameter Real referenceArea(fixed=false),wallArea(fixed=false),referenceVolume(fixed=false),
    wallVolume(fixed=false),minimumCellVolume(fixed=false),minHeight(fixed=false),maxHeight(fixed=false);
protected
  impure function measure
    input String path,wallPath;
    input Real thickness;
    input Boolean limitThickness;
    output Integer cells,shells;
    output Real referenceArea,wallArea,referenceVolume,wallVolume,minimumVolume,minHeight,maxHeight;
  protected
    Real triangles[:,3,3],points[:,3],volumes[:],displacement[3],height;
    Integer faces[:,3],np;
    Meshing.PolyMesh mesh;
  algorithm
    triangles:=Meshing.readAsciiSTL(path);
    (points,faces):=Meshing.indexSurface(triangles);
    mesh:=Meshing.surfacePrismMesh(points,faces,thickness*{0.25,0.56,1},limitThickness=limitThickness);
    (referenceArea,wallArea,referenceVolume,wallVolume):=Meshing.bodySurfaceMetrics(mesh,triangles);
    cells:=mesh.nCells;
    shells:=Meshing.bodyComponentCount(mesh);
    volumes:=Meshing.cellVolumes(mesh,true);
    minimumVolume:=min(volumes);
    np:=size(points,1);
    maxHeight:=0;
    minHeight:=thickness;
    for i in 1:np loop
      displacement:=mesh.points[i+3*np,:]-mesh.points[i,:];
      height:=sqrt(displacement*displacement);
      minHeight:=min(minHeight,height);
      maxHeight:=max(maxHeight,height);
    end for;
    if wallPath<>"" then Meshing.writeBodySTL(wallPath,mesh); end if;
  end measure;
initial algorithm
  (cells,shells,referenceArea,wallArea,referenceVolume,wallVolume,minimumCellVolume,minHeight,maxHeight):=
    measure(surfacePath,wallPath,thickness,limitThickness);
end SurfacePrismStudy;
