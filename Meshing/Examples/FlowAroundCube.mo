within Meshing.Examples;
model FlowAroundCube "Generate a two-layer mesh around a one-metre cube"
  parameter String meshDirectory="cubeCase/constant/polyMesh";
  final parameter Integer cells(fixed=false);
protected
  impure function writeCube
    input String directory;
    output Integer cells;
  protected
    constant Real points[8,3]={{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
    constant Integer faces[12,3]={{1,3,2},{1,4,3},{5,6,7},{5,7,8},{1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,4,8},{3,8,7},{4,1,5},{4,5,8}};
    Real triangles[12,3,3],volumes[:];
    Meshing.PolyMesh mesh;
  algorithm
    for f in 1:12 loop
      for j in 1:3 loop
        triangles[f,j,:]:=points[faces[f,j],:];
      end for;
    end for;
    mesh:=Meshing.generate({-1,-1,-1},{3,3,3},{8,8,8},1,triangles,0);
    mesh:=Meshing.addWallLayers(mesh,2,0.04);
    volumes:=Meshing.cellVolumes(mesh);
    assert(abs(sum(volumes)-63)<1e-8,"Incorrect volume around the cube");
    Meshing.writePolyMesh(directory,mesh);
    cells:=mesh.nCells;
  end writeCube;
initial algorithm
  cells:=writeCube(meshDirectory);
  annotation(experiment(StopTime=0),Documentation(info="<html><p>Simulate once in a fresh working directory.
The cube spans zero to one metre on each axis. The expected fluid volume is
63 cubic metres. To rerun, choose a new meshDirectory or move the previous output.</p></html>"));
end FlowAroundCube;
