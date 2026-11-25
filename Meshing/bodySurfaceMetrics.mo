within Meshing;
function bodySurfaceMetrics "Compare the meshed wall with a closed, oriented triangle surface"
  input PolyMesh mesh;
  input Real triangles[:,3,3](each unit="m")
    "Wind each shell outward from the solid, including cavity boundaries";
  input Integer wallPatch=7;
  output Real referenceArea(unit="m2");
  output Real meshedArea(unit="m2");
  output Real referenceVolume(unit="m3");
  output Real meshedVolume(unit="m3");
protected
  Real origin[3],centre[3],a[3],b[3],areaVector[3];
  Integer nextCorner;
algorithm
  assert(size(triangles,1)>0,"Surface comparison requires reference triangles");
  assert(wallPatch>0,"Choose a closed boundary patch for the surface comparison");
  origin:=triangles[1,1,:];
  referenceArea:=0;
  meshedArea:=0;
  referenceVolume:=0;
  meshedVolume:=0;
  for f in 1:size(triangles,1) loop
    a:=triangles[f,2,:]-triangles[f,1,:];
    b:=triangles[f,3,:]-triangles[f,1,:];
    areaVector:=0.5*cross(a,b);
    referenceArea:=referenceArea+sqrt(areaVector*areaVector);
    referenceVolume:=referenceVolume+(triangles[f,1,:]-origin)*areaVector/3;
  end for;
  for f in 1:mesh.nFaces loop
    if mesh.patch[f]==wallPatch then
      centre:=zeros(3);
      for j in mesh.first[f]:mesh.first[f+1]-1 loop
        centre:=centre+(mesh.points[mesh.corners[j],:]-origin);
      end for;
      centre:=centre/(mesh.first[f+1]-mesh.first[f]);
      // Use the same centroid fan as cellVolumes for warped mesh faces.
      for j in mesh.first[f]:mesh.first[f+1]-1 loop
        nextCorner:=if j+1==mesh.first[f+1] then mesh.first[f] else j+1;
        a:=(mesh.points[mesh.corners[j],:]-origin)-centre;
        b:=(mesh.points[mesh.corners[nextCorner],:]-origin)-centre;
        areaVector:=0.5*cross(a,b);
        meshedArea:=meshedArea+sqrt(areaVector*areaVector);
        // The boundary normal points out of the fluid and into the solid.
        meshedVolume:=meshedVolume-centre*areaVector/3;
      end for;
    end if;
  end for;
  assert(meshedArea>0,"No faces on the selected wall patch");
end bodySurfaceMetrics;
