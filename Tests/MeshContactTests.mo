within Tests;
model MeshContactTests "Face, edge and back-side contact with a two-sided triangular panel"
  parameter MultiBodyLie.Geometry.TriangleMesh panel(nVertices=3,nFaces=1,
    vertices=[0,0,0;1,0,0;0,1,0],faces=[1,2,3]);
  input Real height(unit="m",start=.1);
  MultiBodyLie.Collision.ContactPoint top,edgePoint,back;
equation
  top=MultiBodyLie.Geometry.sphereMesh({0,0,height},{.25,.25,0},.2,zeros(3),identity(3),panel);
  edgePoint=MultiBodyLie.Geometry.sphereMesh({-.1,.2,0},zeros(3),.05,zeros(3),identity(3),panel);
  back=MultiBodyLie.Geometry.sphereMesh({.2,.2,-.1},zeros(3),.05,zeros(3),identity(3),panel);
  assert(abs(top.gap-(abs(height)-.2))<1e-9,"Sphere-face gap");
  assert(abs(edgePoint.gap-.05)<1e-9 and edgePoint.n[1]<-.99,"Outside edge contact");
  assert(abs(back.gap-.05)<1e-9 and back.n[3]<-.99,"Back side of a thin panel");
end MeshContactTests;
