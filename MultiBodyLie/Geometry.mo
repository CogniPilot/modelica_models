within MultiBodyLie;
package Geometry "Mesh contact and independent visual attachments on a rigid body's frame"
  record TriangleMesh "Indexed triangle shell in local coordinates, with one-based vertex indices"
    parameter Integer nVertices(min=3)=3;
    parameter Integer nFaces(min=1)=1;
    Real vertices[nVertices,3](each unit="m");
    Integer faces[nFaces,3];
    Boolean closedConvex=false "True for a closed convex solid with all faces wound outward";
  end TriangleMesh;

  function closestTriangle "Nearest point on a nondegenerate triangle in the same frame as p"
    input Real p[3],a[3],b[3],c[3];
    output Real q[3];
  protected
    Real normal[3],edge[3],start[3],candidate[3],delta[3],best,distance,t;
  algorithm
    normal:=cross(b-a,c-a);
    assert(normal*normal>1e-24,"Degenerate contact triangle");
    q:=p-normal*((p-a)*normal)/(normal*normal);
    if cross(b-a,q-a)*normal<0 or cross(c-b,q-b)*normal<0 or cross(a-c,q-c)*normal<0 then
      q:=a;delta:=p-a;best:=delta*delta;
      for i in 1:3 loop
        start:=if i==1 then a elseif i==2 then b else c;
        edge:=(if i==1 then b elseif i==2 then c else a)-start;
        t:=min(1,max(0,((p-start)*edge)/(edge*edge)));
        candidate:=start+t*edge;delta:=p-candidate;distance:=delta*delta;
        if distance<best then q:=candidate;best:=distance;end if;
      end for;
    end if;
  end closestTriangle;

  function sphereMesh "Contact between a body sphere and a static mesh, resolved in world coordinates"
    input Real originA[3](each unit="m"),centreA[3](each unit="m"),radiusA(unit="m");
    input Real originB[3](each unit="m"),TB[3,3] "World to mesh rotation";
    input TriangleMesh mesh;
    output Collision.ContactPoint point;
  protected
    Real p[3],a[3],b[3],c[3],q[3],delta[3],normal[3],nearest[3],faceNormal[3],n[3];
    Real squared,best,distance;
    Boolean inside;
  algorithm
    p:=TB*((originA-originB)+centreA);
    best:=Modelica.Constants.inf;nearest:=zeros(3);faceNormal:={0,0,1};inside:=mesh.closedConvex;
    for i in 1:mesh.nFaces loop
      a:=mesh.vertices[mesh.faces[i,1],:];b:=mesh.vertices[mesh.faces[i,2],:];c:=mesh.vertices[mesh.faces[i,3],:];
      normal:=cross(b-a,c-a);inside:=inside and (p-a)*normal<=0;
      q:=closestTriangle(p,a,b,c);delta:=p-q;squared:=delta*delta;
      if squared<best then
        best:=squared;nearest:=q;faceNormal:=normal/sqrt(normal*normal);
      end if;
    end for;
    distance:=sqrt(best);
    n:=transpose(TB)*(if distance>1e-12 then (if inside then -1 else 1)*(p-nearest)/distance else faceNormal);
    point:=Collision.ContactPoint(gap=(if inside then -distance else distance)-radiusA,n=n,
      armA=centreA-radiusA*n,armB=transpose(TB)*nearest);
  end sphereMesh;

  model StaticMeshContact "Any number of guard spheres on frame_a against a fixed triangle mesh"
    parameter Integer n(min=1)=1;
    parameter Real centre[n,3](each unit="m")=zeros(n,3),radius[n](each unit="m",each min=0);
    parameter TriangleMesh mesh;
    parameter Real originWorld[3](each unit="m")=zeros(3),worldToMesh[3,3]=identity(3);
    parameter Collision.Material material;
    Interfaces.Frame_a frame_a;
    output Real forceWorld[3](each unit="N"),torqueWorld[3](each unit="N.m");
  protected
    Forces.WorldForceAndTorque wrench(resolveInFrame=Types.ResolveInFrameB.world,animation=false);
    Collision.ContactPoint points[n];
    Real force[n,3],torque[n,3];
  equation
    connect(frame_a,wrench.frame_b);
    for i in 1:n loop
      points[i]=sphereMesh(frame_a.r_0,transpose(frame_a.R.T)*centre[i,:],radius[i],originWorld,worldToMesh,mesh);
      force[i,:]=Collision.contactForce(points[i],transpose(frame_a.R.T)*frame_a.v+cross(transpose(frame_a.R.T)*frame_a.R.w,points[i].armA),material);
      torque[i,:]=cross(points[i].armA,force[i,:]);
    end for;
    for k in 1:3 loop
      forceWorld[k]=sum(force[:,k]);torqueWorld[k]=sum(torque[:,k]);
    end for;
    wrench.force=forceWorld;wrench.torque=torqueWorld;
  end StaticMeshContact;

  model VisualShapes "Massless visual instances attached to one frame, with individual poses, scales and asset URIs"
    parameter Integer n(min=0)=0;
    parameter String uri[n] "One mesh asset per instance";
    parameter Integer color[n,3](each min=0,each max=255)=fill(155,n,3)
      "Visible RGB per visual instance, in red, green, blue channel order";
    input Real bodyTemperature(unit="K", min=0)=293.15
      "Default surface temperature for shapes on this frame";
    input Real temperature[n](each unit="K", each min=0)=fill(bodyTemperature,n)
      "Surface temperature of each shape; may be bound to a thermal model";
    parameter Real offset[n,3](each unit="m")=zeros(n,3),scale[n,3]=ones(n,3);
    parameter Real bodyToVisual[n,3,3]={identity(3) for i in 1:n};
    Interfaces.Frame_a frame_a;
    output Real positionWorld[n,3](each unit="m"),worldToVisual[n,3,3];
  equation
    frame_a.f=zeros(3);frame_a.t=zeros(3);frame_a.dv=zeros(3);frame_a.pA=zeros(6);frame_a.IA=zeros(21);
    for i in 1:n loop
      positionWorld[i,:]=frame_a.r_0+transpose(frame_a.R.T)*offset[i,:];
      worldToVisual[i,:,:]=bodyToVisual[i,:,:]*frame_a.R.T;
    end for;
  end VisualShapes;
end Geometry;
