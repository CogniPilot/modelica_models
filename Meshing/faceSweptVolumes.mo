within Meshing;
function faceSweptVolumes "Volume swept by each oriented face during a linear vertex motion"
  input PolyMesh mesh "Geometry at the beginning of the interval";
  input Real endPoints[mesh.nPoints,3](each unit="m") "Same vertices and connectivity at the end";
  output Real swept[mesh.nFaces](each unit="m3") "Positive for motion out of the owner";
protected
  Real c0[3],c1[3],a0[3],a1[3],b0[3],b1[3],meanMotion[3],integratedArea[3];
  Integer a,b,first,last;
algorithm
  swept:=zeros(mesh.nFaces);
  for f in 1:mesh.nFaces loop
    first:=mesh.first[f];
    last:=mesh.first[f+1]-1;
    c0:=zeros(3);
    c1:=zeros(3);
    for j in first:last loop
      a:=mesh.corners[j];
      c0:=c0+mesh.points[a,:];
      c1:=c1+endPoints[a,:];
    end for;
    c0:=c0/(last-first+1);
    c1:=c1/(last-first+1);
    for j in first:last loop
      a:=mesh.corners[j];
      b:=mesh.corners[if j==last then first else j+1];
      a0:=mesh.points[a,:]-c0;
      a1:=endPoints[a,:]-c1;
      b0:=mesh.points[b,:]-c0;
      b1:=endPoints[b,:]-c1;
      // Simpson integration is exact for the quadratic area vector. The
      // centroid fan matches the face convention used in cellVolumes.
      integratedArea:=(cross(a0,b0)+cross(a0+a1,b0+b1)+cross(a1,b1))/12;
      meanMotion:=(c1-c0+endPoints[a,:]-mesh.points[a,:]+endPoints[b,:]-mesh.points[b,:])/3;
      swept[f]:=swept[f]+meanMotion*integratedArea;
    end for;
  end for;
  annotation(Documentation(info="<html><p>Sum outgoing face sweeps in a cell to recover its volume
change. Divide by the time interval for the mean mesh volume flux. This operates on
prescribed coordinates and unchanging connectivity. A coupled solver supplies mesh
motion, interface loads and its temporal discretization.</p></html>"));
end faceSweptVolumes;
