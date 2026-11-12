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
      assert(abs(f[i]-fGround[i])<=0,"The shared force law must give the ground wrench's force bit for bit");
      assert(abs(tau[i]-tauGround[i])<=0,"The moment of the shared law's force must be the ground wrench's torque bit for bit");
      assert(abs(fClear[i])<=0 and abs(fGroundClear[i])<=0 and abs(tauGroundClear[i])<=0,
        "A sphere clear of the ground carries exactly no force on either path");
      assert(abs(point.armB[i]+point.gap*point.n[i]-(r0[i]+point.armA[i]))<1e-15,
        "The witness point on the plane is the foot of the sphere's point");
    end for;
  end PlaneSphereAgainstGroundWrench;

  model SceneTables "A scene's default tables are the ground plane, one material and one group"
    import MultiBodyLie.Collision;
    Collision.Scene scene;
  equation
    assert(scene.nStatic==1 and scene.staticShapes[1].kind==Collision.ShapeKind.Plane,
      "The default scene is the ground plane");
    assert(Collision.maxPoints[Collision.ShapeKind.Plane,Collision.ShapeKind.Sphere]==1
      and Collision.maxPoints[Collision.ShapeKind.Box,Collision.ShapeKind.Box]==0,
      "The pair table gives one point for plane-sphere and marks box-box as unsupported");
  end SceneTables;

  model Suite "Plane-sphere pair against the ground wrench; scene tables"
    PlaneSphereAgainstGroundWrench planeSphere;
    SceneTables tables;
    annotation(experiment(StartTime=0,StopTime=0,Tolerance=1e-8));
  end Suite;
end CollisionTests;
