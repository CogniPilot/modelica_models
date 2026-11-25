within Meshing;
impure function writeBodySTL "Export the wall of an exterior fluid mesh, wound outward from the solid"
  input String path;
  input PolyMesh mesh;
  input Integer wallPatch=7;
protected
  function coordinates
    input Real p[3];
    output String row;
  algorithm
    row:=String(p[1],significantDigits=17)+" "+String(p[2],significantDigits=17)+" "+String(p[3],significantDigits=17);
  end coordinates;
  Real centre[3],a[3],b[3],normal[3],norm;
  Integer start,last;
algorithm
  assert(wallPatch>0 and max(if mesh.patch[f]==wallPatch then 1 else 0 for f in 1:mesh.nFaces)==1,"Wall patch is empty");
  if Modelica.Utilities.Files.exist(path) then Modelica.Utilities.Files.removeFile(path); end if;
  Modelica.Utilities.Streams.print("solid wall",path);
  for f in 1:mesh.nFaces loop
    if mesh.patch[f]==wallPatch then
      start:=mesh.first[f];
      last:=mesh.first[f+1]-1;
      centre:=zeros(3);
      for j in start:last loop centre:=centre+mesh.points[mesh.corners[j],:]; end for;
      centre:=centre/(last-start+1);
      for j in start:last loop
        a:=mesh.points[mesh.corners[if j==last then start else j+1],:];
        b:=mesh.points[mesh.corners[j],:];
        normal:=cross(a-centre,b-centre);
        norm:=sqrt(normal*normal);
        assert(norm>0,"Degenerate wall triangle at face "+String(f));
        Modelica.Utilities.Streams.print("facet normal "+coordinates(normal/norm)+"\nouter loop\nvertex "+
          coordinates(centre)+"\nvertex "+coordinates(a)+"\nvertex "+coordinates(b)+"\nendloop\nendfacet",path);
      end for;
    end if;
  end for;
  Modelica.Utilities.Streams.print("endsolid wall",path);
  Modelica.Utilities.Streams.close(path);
end writeBodySTL;
