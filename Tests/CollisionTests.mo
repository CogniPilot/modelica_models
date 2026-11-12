within Tests;
package CollisionTests "Collision scene tables, the plane-sphere pair and the shared contact force law"
  model PlaneSphereAgainstGroundWrench "The plane-sphere pair with the shared force law gives the force and torque of Contact.groundWrench"
    import MultiBodyLie.Collision;
    parameter Real r0[3](each unit="m")={0.4,-0.3,-0.02} "Origin of the owning frame, world";
    parameter Real T[3,3]=MultiBodyLie.Frames.axesRotationsT({1,2,3},{0.2,-0.3,0.5})
      "World-to-frame rotation of a tilted body";
    parameter Real w[3](each unit="rad/s")={0.7,-1.1,0.4} "Angular velocity, resolved in the frame";
    parameter Real v0[3](each unit="m/s")={0.03,-0.04,-0.05} "Velocity of the frame origin, world";
    parameter Real centre[1,3](each unit="m")={{0.02,-0.01,0.005}} "Sphere centre in the frame";
    parameter Real radius(unit="m")=0.012;
    parameter Collision.Material material(k=1e4,d=0.5,mu=0.8,eps=1e-4,v_eps=0.01);
    parameter Real lift(unit="m")=1 "Height added for the case that is clear of the ground";
    parameter Real planePoint[3](each unit="m")=zeros(3) "A point of the ground plane";
    parameter Real planeNormal[3]={0,0,1} "Normal of the ground plane";
    Real arm[3](each unit="m") "Sphere centre relative to the frame origin, world axes";
    Collision.ContactPoint point "The pair's one contact point";
    Collision.ContactPoint pointClear "The same sphere lifted clear of the ground";
    Real v[3](each unit="m/s") "Velocity of the witness point, world";
    Real f[3](each unit="N");
    Real tau[3](each unit="N.m");
    Real fClear[3](each unit="N");
    Real fGround[3](each unit="N");
    Real tauGround[3](each unit="N.m");
    Real fGroundClear[3](each unit="N");
    Real tauGroundClear[3](each unit="N.m");
  equation
    arm=transpose(T)*centre[1,:];
    point=Collision.planeSphere(planePoint,planeNormal,r0,arm,radius);
    pointClear=Collision.planeSphere(planePoint,planeNormal,r0+{0,0,lift},arm,radius);
    v=v0+cross(transpose(T)*w,point.armA);
    f=Collision.contactForce(point,v,material);
    tau=cross(point.armA,f);
    fClear=Collision.contactForce(pointClear,v,material);
    (fGround,tauGround)=MultiBodyLie.Contact.groundWrench(r0,T,w,v0,centre,{radius},
      material.k,material.d,material.mu,material.eps,material.v_eps);
    (fGroundClear,tauGroundClear)=MultiBodyLie.Contact.groundWrench(r0+{0,0,lift},T,w,v0,centre,{radius},
      material.k,material.d,material.mu,material.eps,material.v_eps);
    assert(point.gap<0 and fGround[3]>1,"The fixture must press the sphere into the ground");
    for i in 1:3 loop
      assert(abs(f[i]-fGround[i])<=0,"The shared force law must give exactly the ground wrench's force on this fixture");
      assert(abs(tau[i]-tauGround[i])<=0,"The moment of the shared law's force must be exactly the ground wrench's torque on this fixture");
      assert(abs(fClear[i])<=0 and abs(fGroundClear[i])<=0 and abs(tauGroundClear[i])<=0,
        "A sphere clear of the ground carries exactly no force on either path");
      assert(abs(point.armB[i]+point.gap*point.n[i]-(r0[i]+point.armA[i]))<1e-15,
        "The witness point on the plane is the foot of the sphere's point");
    end for;
  end PlaneSphereAgainstGroundWrench;

  model SoleAgainstGroundWrench "Four sole spheres through the scene tables give the force and torque of Contact.groundWrench"
    import MultiBodyLie.Collision;
    parameter Real footLength(unit="m")=0.07;
    parameter Real footWidth(unit="m")=0.04;
    parameter Real radius(unit="m")=0.003;
    parameter Real centre[4,3](each unit="m")={{-footLength/2,-footWidth/2,0},{-footLength/2,footWidth/2,0},
      {footLength/2,-footWidth/2,0},{footLength/2,footWidth/2,0}} "Sole sphere centres in the foot frame";
    parameter Collision.Material material(k=1e4,d=0.5,mu=0.8,eps=1e-4,v_eps=0.01);
    parameter Collision.ShapeKind kind[4]=fill(Collision.ShapeKind.Sphere,4) "The sole as a table of spheres";
    parameter Real size[4,3](each unit="m")={{radius,0,0} for i in 1:4};
    parameter Collision.ShapeKind groundKind[1]={Collision.ShapeKind.Plane} "The world: the ground plane";
    parameter Real groundR[1,3](each unit="m")=zeros(1,3);
    parameter Real groundT[1,3,3]={identity(3)};
    parameter Boolean collides[1,1]={{true}};
    parameter Real r0[3](each unit="m")={0.1,0.2,0.004} "Foot frame origin, world: the heel side is pressed in, the toe side is clear";
    parameter Real T[3,3]=MultiBodyLie.Frames.axesRotationsT({1,2,3},{0.05,0.12,-0.4}) "World-to-foot rotation";
    parameter Real w[3](each unit="rad/s")={0.3,-0.9,0.2};
    parameter Real v0[3](each unit="m/s")={0.05,0.02,-0.03};
    Real f[3](each unit="N");
    Real tau[3](each unit="N.m");
    Real fGround[3](each unit="N");
    Real tauGround[3](each unit="N.m");
  equation
    (f,tau)=Collision.sceneWrench(r0,T,w,v0,kind,centre,{identity(3) for i in 1:4},size,ones(4),ones(4),groundKind,groundR,groundT,{1},
      {material.k},{material.d},{material.mu},{material.eps},{material.v_eps},collides);
    (fGround,tauGround)=MultiBodyLie.Contact.groundWrench(r0,T,w,v0,centre,fill(radius,4),
      material.k,material.d,material.mu,material.eps,material.v_eps);
    assert(fGround[3]>0.1,"The fixture must press part of the sole into the ground");
    for i in 1:3 loop
      assert(abs(f[i]-fGround[i])<=0,"The scene tables must give exactly the ground wrench's force on this fixture");
      assert(abs(tau[i]-tauGround[i])<=0,"The scene tables must give exactly the ground wrench's torque on this fixture");
    end for;
  end SoleAgainstGroundWrench;

  model CapsuleAndBoxAgainstGroundWrench "A capsule and a box through the scene tables give the ground wrench of their end spheres and corners, to rounding"
    import MultiBodyLie.Collision;
    parameter Collision.Material material(k=1e4,d=0.5,mu=0.8,eps=1e-4,v_eps=0.01);
    parameter Real capsuleR[3](each unit="m")={0.03,-0.01,0.002} "Capsule centre in the body frame";
    parameter Real capsuleT[3,3]=MultiBodyLie.Frames.axesRotationsT({3,2,1},{0.6,0.15,0}) "Body-to-capsule rotation";
    parameter Real capsuleRadius(unit="m")=0.004;
    parameter Real capsuleHalf(unit="m")=0.02;
    parameter Real boxR[3](each unit="m")={-0.02,0.015,0.004} "Box centre in the body frame";
    parameter Real boxT[3,3]=MultiBodyLie.Frames.axesRotationsT({1,2,3},{0.1,-0.2,0.7}) "Body-to-box rotation";
    parameter Real boxHalf[3](each unit="m")={0.012,0.008,0.005};
    parameter Real r0[3](each unit="m")={0.1,0.2,-0.004} "Body frame origin, world";
    parameter Real T[3,3]=MultiBodyLie.Frames.axesRotationsT({1,2,3},{0.04,-0.07,0.9}) "World-to-body rotation";
    parameter Real w[3](each unit="rad/s")={0.4,0.6,-0.3};
    parameter Real v0[3](each unit="m/s")={-0.02,0.05,-0.04};
    parameter Real spheres[10,3](each unit="m")={
      capsuleR-capsuleHalf*capsuleT[1,:],
      capsuleR+capsuleHalf*capsuleT[1,:],
      boxR+transpose(boxT)*{-boxHalf[1],-boxHalf[2],-boxHalf[3]},
      boxR+transpose(boxT)*{-boxHalf[1],-boxHalf[2],boxHalf[3]},
      boxR+transpose(boxT)*{-boxHalf[1],boxHalf[2],-boxHalf[3]},
      boxR+transpose(boxT)*{-boxHalf[1],boxHalf[2],boxHalf[3]},
      boxR+transpose(boxT)*{boxHalf[1],-boxHalf[2],-boxHalf[3]},
      boxR+transpose(boxT)*{boxHalf[1],-boxHalf[2],boxHalf[3]},
      boxR+transpose(boxT)*{boxHalf[1],boxHalf[2],-boxHalf[3]},
      boxR+transpose(boxT)*{boxHalf[1],boxHalf[2],boxHalf[3]}}
      "The two capsule ends, then the eight box corners, in the body frame, written out independently";
    Real f[3](each unit="N");
    Real tau[3](each unit="N.m");
    Real fGround[3](each unit="N");
    Real tauGround[3](each unit="N.m");
    Real fCapsule[3](each unit="N") "The capsule's two end spheres alone";
    Real tauCapsule[3](each unit="N.m");
    Real fBox[3](each unit="N") "The box's eight corners alone";
    Real tauBox[3](each unit="N.m");
  equation
    (f,tau)=Collision.sceneWrench(r0,T,w,v0,{Collision.ShapeKind.Capsule,Collision.ShapeKind.Box},{capsuleR,boxR},
      {capsuleT,boxT},{{capsuleRadius,capsuleHalf,0},boxHalf},{1,1},{1,1},{Collision.ShapeKind.Plane},zeros(1,3),
      {identity(3)},{1},{material.k},{material.d},{material.mu},{material.eps},{material.v_eps},{{true}});
    (fGround,tauGround)=MultiBodyLie.Contact.groundWrench(r0,T,w,v0,spheres,cat(1,fill(capsuleRadius,2),zeros(8)),
      material.k,material.d,material.mu,material.eps,material.v_eps);
    (fCapsule,tauCapsule)=MultiBodyLie.Contact.groundWrench(r0,T,w,v0,spheres[1:2,:],fill(capsuleRadius,2),
      material.k,material.d,material.mu,material.eps,material.v_eps);
    (fBox,tauBox)=MultiBodyLie.Contact.groundWrench(r0,T,w,v0,spheres[3:10,:],zeros(8),
      material.k,material.d,material.mu,material.eps,material.v_eps);
    assert(fCapsule[3]>0.1 and fBox[3]>0.1,"The fixture must press both shapes into the ground");
    for i in 1:3 loop
      // The sphere centres are computed twice, here and in the library, so the comparison is to rounding.
      assert(abs(f[i]-fGround[i])<1e-12*max(1,abs(fGround[i])),"The capsule and box must give the ground wrench's force of their end spheres and corners");
      assert(abs(tau[i]-tauGround[i])<1e-12*max(1,abs(tauGround[i])),"The capsule and box must give the ground wrench's torque of their end spheres and corners");
      assert(abs(fCapsule[i]+fBox[i]-fGround[i])<1e-12*max(1,abs(fGround[i])),"The two shapes' forces add up");
    end for;
  end CapsuleAndBoxAgainstGroundWrench;

  model FallingBodies "A body with a BodyShapes table moves as the same body with a GroundContactBody"
    import MultiBodyLie.Collision;
    parameter Real radius(unit="m")=0.01;
    parameter Real centre[2,3](each unit="m")={{0.02,0,0},{-0.02,0.01,0}};
    inner MultiBodyLie.World world(n={0,0,-1},enableAnimation=false);
    inner Collision.Scene scene(k={1e4});
    MultiBodyLie.Parts.Body table(m=0.2,r_CM={0.001,0.002,0},I_11=1e-4,I_22=2e-4,I_33=1.5e-4,
      r_0(start={0,0,0.02},each fixed=true),v_0(start={0.1,0,0},each fixed=true),
      angles_fixed=true,w_0_fixed=true,w_0_start={0.5,-0.2,0.1});
    MultiBodyLie.Parts.Body spheres(m=0.2,r_CM={0.001,0.002,0},I_11=1e-4,I_22=2e-4,I_33=1.5e-4,
      r_0(start={0,0,0.02},each fixed=true),v_0(start={0.1,0,0},each fixed=true),
      angles_fixed=true,w_0_fixed=true,w_0_start={0.5,-0.2,0.1});
    Collision.BodyShapes shapes(n=2,r=centre,size={{radius,0,0} for i in 1:2});
    MultiBodyLie.Contact.GroundContactBody ground(n=2,centre=centre,radius=fill(radius,2),k=1e4);
  equation
    connect(table.frame_a,shapes.frame_a);
    connect(spheres.frame_a,ground.frame_a);
    shapes.v0=table.v_0;
    ground.v0=spheres.v_0;
    for i in 1:3 loop
      assert(abs(table.r_0[i]-spheres.r_0[i])<1e-9,"The two bodies must stay at the same position");
      assert(abs(shapes.f[i]-ground.f[i])<1e-6*max(1,abs(ground.f[i])),"The two contact models must apply the same force");
    end for;
  end FallingBodies;

  model SceneTables "A scene's default tables are the ground plane, one material and one group"
    import MultiBodyLie.Collision;
    Collision.Scene scene;
  equation
    assert(scene.nStatic==1 and scene.staticKind[1]==Collision.ShapeKind.Plane,
      "The default scene is the ground plane");
    assert(Collision.maxPoints[Collision.ShapeKind.Plane,Collision.ShapeKind.Sphere]==1
      and Collision.maxPoints[Collision.ShapeKind.Plane,Collision.ShapeKind.Box]==8
      and Collision.maxPoints[Collision.ShapeKind.Box,Collision.ShapeKind.Box]==0,
      "The pair table gives one point for plane-sphere, eight for plane-box and marks box-box as unsupported");
  end SceneTables;

  model Suite "Plane pairs and the scene tables against the ground wrench; a falling body on both contact models"
    PlaneSphereAgainstGroundWrench planeSphere;
    SoleAgainstGroundWrench sole;
    CapsuleAndBoxAgainstGroundWrench capsuleAndBox;
    FallingBodies falling;
    SceneTables tables;
    annotation(experiment(StartTime=0,StopTime=0.2,Interval=0.002,Tolerance=1e-8));
  end Suite;
end CollisionTests;
