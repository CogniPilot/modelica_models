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
    (f,tau)=Collision.sceneWrench(r0,T,w,v0,kind,centre,{identity(3) for i in 1:4},size,ones(4),ones(4),groundKind,groundR,groundT,{1},{1},
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
      {identity(3)},{1},{1},{material.k},{material.d},{material.mu},{material.eps},{material.v_eps},{{true}});
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

  model SpherePairs "A sphere against a sphere, a capsule and a box: analytic gaps, normals and witness points, and a balanced pair of wrenches"
    import MultiBodyLie.Collision;
    parameter Real originA[3](each unit="m")={1024,-2048,512} "Frame of the sphere, far from the world origin";
    parameter Real offset[3](each unit="m")={0.25,0.125,-0.1875}
      "From the sphere's frame to the other shape's frame; binary fractions, so originA+offset rounds nothing";
    parameter Real centreA[3](each unit="m")={0.01,0.02,-0.01} "Sphere centre in its frame, world axes";
    parameter Real radiusA(unit="m")=0.03;
    parameter Real radiusB(unit="m")=0.02 "Radius of the other sphere and of the capsule";
    parameter Real dir[3]={2,-1,2}/3 "Unit vector from the sphere towards the other sphere";
    parameter Real axis[3]={1,2,2}/3 "Unit vector along the capsule";
    parameter Real side[3]={2,1,-2}/3 "Unit vector across the capsule";
    parameter Real halfLength(unit="m")=0.05 "Half length of the capsule";
    parameter Real TB[3,3]=MultiBodyLie.Frames.axesRotationsT({1,2,3},{0.2,-0.3,0.5}) "World-to-box rotation";
    parameter Real halfB[3](each unit="m")={0.05,0.03,0.02} "Half extents of the box";
    parameter Real centreB[3](each unit="m")={0.02,-0.01,0.03} "Middle of the capsule, and centre of the box, in their frame";
    parameter Collision.Material material(k=1e4,d=0.5,mu=0.8,eps=1e-4,v_eps=0.01);
    parameter Real vA[3](each unit="m/s")={0.03,-0.04,-0.05};
    parameter Real wA[3](each unit="rad/s")={0.7,-1.1,0.4};
    parameter Real vB[3](each unit="m/s")={-0.02,0.01,0.06};
    parameter Real wB[3](each unit="rad/s")={-0.3,0.9,0.2};
    Real originB[3](each unit="m") "Frame of the other shape";
    Real L(unit="m") "Distance that shrinks with time: separated, touching, then overlapping";
    Collision.ContactPoint spheres "Sphere a against sphere b";
    Collision.ContactPoint spheresFlipped "Sphere b against sphere a";
    Collision.ContactPoint beside "Sphere beside the capsule's segment";
    Collision.ContactPoint beyond "Sphere beyond the end of the capsule's segment";
    Collision.ContactPoint onFace "Sphere over a face of the box";
    Collision.ContactPoint onCorner "Sphere off a corner of the box";
    Collision.ContactPoint inside "Sphere centre inside the box, nearest its +x face";
    Real fA[3](each unit="N");
    Real tauA[3](each unit="N.m");
    Real fB[3](each unit="N");
    Real tauB[3](each unit="N.m");
    Real moment[3](each unit="N.m") "Total moment of the pair about the origin of a's frame";
  equation
    originB=originA+offset;
    L=0.08-0.2*time;
    spheres=Collision.sphereSphere(originA,centreA,radiusA,originB,centreA+L*dir-offset,radiusB);
    spheresFlipped=Collision.sphereSphere(originB,centreA+L*dir-offset,radiusB,originA,centreA,radiusA);
    // The capsule's middle is placed so that the sphere centre is at 0.02 m, then 0.09 m, along its axis and L across it.
    beside=Collision.sphereCapsule(originA,centreA,radiusA,originB,centreA-0.02*axis-L*side-offset,axis,halfLength,radiusB);
    beyond=Collision.sphereCapsule(originA,centreA,radiusA,originB,centreA-0.09*axis-L*side-offset,axis,halfLength,radiusB);
    // The box's centre is placed so that the sphere centre is at the given point of the box frame.
    onFace=Collision.sphereBox(originA,centreA,radiusA,originB,centreA-transpose(TB)*{halfB[1]+L,0.01,0.005}-offset,TB,halfB);
    onCorner=Collision.sphereBox(originA,centreA,radiusA,originB,
      centreA-transpose(TB)*{halfB[1]+L,halfB[2]+2*L,halfB[3]+2*L}-offset,TB,halfB);
    inside=Collision.sphereBox(originA,centreA,radiusA,originB,centreA-transpose(TB)*{halfB[1]-0.004,0.001*time,0}-offset,TB,halfB);
    (fA,tauA,fB,tauB)=Collision.pairWrenches(spheres,vA,wA,vB,wB,material);
    moment=tauA+tauB+cross(originB-originA,fB);
    assert(abs(spheres.gap-(L-radiusA-radiusB))<1e-12,"Two spheres are apart by the distance of their centres less their radii");
    assert(abs(beside.gap-(L-radiusA-radiusB))<1e-12,"A sphere beside a capsule is apart by its distance from the axis less the radii");
    assert(abs(beyond.gap-(sqrt(L^2+0.04^2)-radiusA-radiusB))<1e-12,"A sphere beyond a capsule's end is apart by its distance from the end less the radii");
    assert(abs(onFace.gap-(L-radiusA))<1e-12,"A sphere over a box face is apart by its height over the face less its radius");
    assert(abs(onCorner.gap-(3*L-radiusA))<1e-12,"A sphere off a box corner is apart by its distance from the corner less its radius");
    assert(abs(inside.gap-(-0.004-radiusA))<1e-12,"A sphere centre inside a box overlaps by its depth under the nearest face and its radius");
    assert(time<0.16 or fA*spheres.n>1,"The spheres must press on each other at the end of the run");
    for i in 1:3 loop
      assert(abs(spheres.n[i]+dir[i])<1e-12,"The normal of two spheres runs from b to a along their centres");
      assert(abs(beside.n[i]-side[i])<1e-12,"The normal beside a capsule is across its axis");
      assert(abs(onFace.n[i]-TB[1,i])<1e-12,"The normal over a box face is the face's axis");
      assert(abs(onCorner.n[i]-(TB[1,i]+2*TB[2,i]+2*TB[3,i])/3)<1e-12,"The normal off a box corner runs from the corner to the sphere centre");
      assert(abs(inside.n[i]-TB[1,i])<1e-12,"The normal of a centre inside a box is the nearest face's axis");
      // The two witness points are apart by the gap along the normal, for every pair.
      assert(abs(spheres.armA[i]-spheres.armB[i]-offset[i]-spheres.gap*spheres.n[i])<1e-12,"sphere-sphere witness points");
      assert(abs(beside.armA[i]-beside.armB[i]-offset[i]-beside.gap*beside.n[i])<1e-12,"sphere-capsule witness points, beside");
      assert(abs(beyond.armA[i]-beyond.armB[i]-offset[i]-beyond.gap*beyond.n[i])<1e-12,"sphere-capsule witness points, beyond the end");
      assert(abs(onFace.armA[i]-onFace.armB[i]-offset[i]-onFace.gap*onFace.n[i])<1e-12,"sphere-box witness points, face");
      assert(abs(onCorner.armA[i]-onCorner.armB[i]-offset[i]-onCorner.gap*onCorner.n[i])<1e-12,"sphere-box witness points, corner");
      assert(abs(inside.armA[i]-inside.armB[i]-offset[i]-inside.gap*inside.n[i])<1e-12,"sphere-box witness points, inside");
      // Exchanging the shapes exchanges the witness points and turns the normal.
      assert(abs(spheresFlipped.n[i]+spheres.n[i])<1e-12 and abs(spheresFlipped.armA[i]-spheres.armB[i])<1e-12
        and abs(spheresFlipped.armB[i]-spheres.armA[i])<1e-12,"The pair with its shapes exchanged is the flipped point");
      assert(abs(fA[i]+fB[i])<=0,"The two forces of a pair are exactly opposite");
      assert(abs(moment[i])<1e-9*max(1,abs(fA*spheres.n)),"The two wrenches of a pair have no total moment");
    end for;
    assert(abs(spheresFlipped.gap-spheres.gap)<1e-12,"The gap does not depend on the order of the shapes");
  end SpherePairs;

  model CapsulePairs "Two capsules crossing, end to side, end to end and lying along each other: analytic gaps, normals and witness points"
    import MultiBodyLie.Collision;
    parameter Real originA[3](each unit="m")={1024,-2048,512} "Frame of capsule a, far from the world origin";
    parameter Real offset[3](each unit="m")={0.25,0.125,-0.1875}
      "From a's frame to b's frame; binary fractions, so originA+offset rounds nothing";
    parameter Real centreA[3](each unit="m")={0.01,0.02,-0.01} "Middle of a's segment in its frame, world axes";
    parameter Real halfA(unit="m")=0.1;
    parameter Real radiusA(unit="m")=0.02;
    parameter Real halfB(unit="m")=0.08;
    parameter Real radiusB(unit="m")=0.03;
    parameter Real x[3]={1,0,0};
    parameter Real z[3]={0,0,1};
    parameter Real slant[3]={0.6,0.8,0} "A unit vector in the horizontal plane, neither along nor across x";
    Real originB[3](each unit="m");
    Real L(unit="m") "Distance of the nearest points: separated, touching, then overlapping";
    Collision.ContactPoint crossing "a along x over b along the slant: both nearest points inside the segments";
    Collision.ContactPoint crossingFlipped "The same pair with the capsules exchanged";
    Collision.ContactPoint endToSide "a upright over b: a's lower end against b's side";
    Collision.ContactPoint endToEnd "a and b in one line: facing ends";
    Collision.ContactPoint along "a over b, both along x, overlapping";
  equation
    originB=originA+offset;
    L=0.08-0.2*time;
    // b's middle is placed so that the nearest points are at 0.015 m on a and -0.025 m on b.
    crossing=Collision.capsuleCapsule(originA,centreA,x,halfA,radiusA,originB,centreA+{0.03,0.02,-L}-offset,slant,halfB,radiusB);
    crossingFlipped=Collision.capsuleCapsule(originB,centreA+{0.03,0.02,-L}-offset,slant,halfB,radiusB,originA,centreA,x,halfA,radiusA);
    endToSide=Collision.capsuleCapsule(originA,centreA,z,halfA,radiusA,originB,centreA+{0,-0.01,-(halfA+L)}-offset,{0,1,0},halfB,radiusB);
    endToEnd=Collision.capsuleCapsule(originA,centreA,x,halfA,radiusA,originB,centreA+{halfA+halfB+L,0,0}-offset,x,halfB,radiusB);
    along=Collision.capsuleCapsule(originA,centreA,x,halfA,radiusA,originB,centreA+{0.05,0,-L}-offset,x,halfB,radiusB);
    assert(abs(crossing.gap-(L-radiusA-radiusB))<1e-12,"Crossing capsules are apart by the distance of their axes less the radii");
    assert(abs(endToSide.gap-(L-radiusA-radiusB))<1e-12,"A capsule end over another capsule's side is apart by its height less the radii");
    assert(abs(endToEnd.gap-(L-radiusA-radiusB))<1e-12,"Capsules in one line are apart by the distance of their facing ends less the radii");
    assert(abs(along.gap-(L-radiusA-radiusB))<1e-12,"Capsules lying along each other are apart by the distance of their axes less the radii");
    assert(abs(crossingFlipped.gap-crossing.gap)<1e-12,"The gap does not depend on the order of the capsules");
    for i in 1:3 loop
      assert(abs(crossing.n[i]-z[i])<1e-12 and abs(endToSide.n[i]-z[i])<1e-12 and abs(along.n[i]-z[i])<1e-12,
        "The normal of a capsule over another runs up from b to a");
      assert(abs(endToEnd.n[i]+x[i])<1e-12,"The normal of capsules in one line runs along the line from b to a");
      // The nearest points themselves: on a at 0.015 m along x, its lower end, its end towards b, and its middle.
      assert(abs(crossing.armA[i]-(centreA[i]+0.015*x[i]-radiusA*z[i]))<1e-12,"crossing: witness point on a");
      assert(abs(endToSide.armA[i]-(centreA[i]-(halfA+radiusA)*z[i]))<1e-12,"end to side: witness point on a");
      assert(abs(endToEnd.armA[i]-(centreA[i]+(halfA+radiusA)*x[i]))<1e-12,"end to end: witness point on a");
      assert(abs(along.armA[i]-(centreA[i]-radiusA*z[i]))<1e-12,"along: witness point on a is under its middle");
      // The two witness points are apart by the gap along the normal.
      assert(abs(crossing.armA[i]-crossing.armB[i]-offset[i]-crossing.gap*crossing.n[i])<1e-12,"crossing: witness points");
      assert(abs(endToSide.armA[i]-endToSide.armB[i]-offset[i]-endToSide.gap*endToSide.n[i])<1e-12,"end to side: witness points");
      assert(abs(endToEnd.armA[i]-endToEnd.armB[i]-offset[i]-endToEnd.gap*endToEnd.n[i])<1e-12,"end to end: witness points");
      assert(abs(along.armA[i]-along.armB[i]-offset[i]-along.gap*along.n[i])<1e-12,"along: witness points");
      assert(abs(crossingFlipped.n[i]+crossing.n[i])<1e-12 and abs(crossingFlipped.armA[i]-crossing.armB[i])<1e-12
        and abs(crossingFlipped.armB[i]-crossing.armA[i])<1e-12,"The pair with its capsules exchanged is the flipped point");
    end for;
  end CapsulePairs;

  function offSurface "How far a contact point on a capsule b is from having a unit normal across the capsule's axis and its witness point on the capsule's surface there"
    input MultiBodyLie.Collision.ContactPoint point "A contact point whose shape b is the capsule";
    input Real onSegment[3](each unit="m") "The point of the capsule's segment the contact is at, as armB is given";
    input Real axis[3] "Unit vector along the capsule";
    input Real radius(unit="m") "Radius of the capsule";
    output Real error "Largest of: the normal's deviation from unit length, its component along the axis, and the distance of armB from onSegment + radius * n";
  protected
    Real w[3](each unit="m") "From the point on the segment to the witness point";
  algorithm
    w:=point.armB-onSegment;
    error:=max({abs(point.n*point.n-1),abs(point.n*axis),abs(w[1]-radius*point.n[1]),abs(w[2]-radius*point.n[2]),
      abs(w[3]-radius*point.n[3])});
  end offSurface;

  model CapsuleEdgeCases "Capsules at a very small angle keep their nearest points, and a centreline that is met gives a normal across the capsule with witness points on the surfaces"
    import MultiBodyLie.Collision;
    parameter Real origin[3](each unit="m")={1024,-2048,512} "Frame of both shapes, far from the world origin";
    parameter Real centre[3](each unit="m")={0.01,0.02,-0.01} "A point in that frame";
    parameter Real x[3]={1,0,0};
    parameter Real z[3]={0,0,1};
    parameter Real slant[3]={0.6,0.8,0} "A unit vector that is least along z";
    parameter Real tilted[3]={1,2,2}/3 "A unit vector that is least along x";
    parameter Real e=5e-7 "Sine of the small angle between two capsules";
    parameter Real d(unit="m")=1e-8 "Distance of their centrelines";
    // a along x and b at the angle asin(e) to it in the horizontal plane, b's middle placed so that the
    // nearest points are strictly inside both segments: at sqrt(1 - e^2) / 2 on a and at 1 / 2 on b, d apart
    // along z. Taken as parallel, the pair would be returned about 2.5e-7 m apart with a normal along y.
    Collision.ContactPoint nearlyParallel "Two capsules of half length 1 and radius 0.02 at that angle";
    // A sphere of radius 0.1 centred on the segment of a capsule of half length 1 and radius 0.2.
    Collision.ContactPoint onAxisZ "At the middle of a capsule along z";
    Collision.ContactPoint onAxisX "At the middle of a capsule along x";
    Collision.ContactPoint onAxisSlant "At the middle of a capsule along the slant";
    Collision.ContactPoint onAxisTilted "At the middle of a tilted capsule";
    Collision.ContactPoint atEnd "At the end of a capsule along z";
    // Two capsules of half length 1 and radii 0.02 and 0.03 whose centrelines meet at their middles.
    Collision.ContactPoint meeting "a along x through b along the slant";
    Collision.ContactPoint collinear "a and b both along z";
  equation
    nearlyParallel=Collision.capsuleCapsule(origin,zeros(3),x,1,0.02,origin,{0,-e/2,d},{sqrt(1-e^2),e,0},1,0.02);
    onAxisZ=Collision.sphereCapsule(origin,centre,0.1,origin,centre,z,1,0.2);
    onAxisX=Collision.sphereCapsule(origin,centre,0.1,origin,centre,x,1,0.2);
    onAxisSlant=Collision.sphereCapsule(origin,centre,0.1,origin,centre,slant,1,0.2);
    onAxisTilted=Collision.sphereCapsule(origin,centre,0.1,origin,centre,tilted,1,0.2);
    atEnd=Collision.sphereCapsule(origin,centre+z,0.1,origin,centre,z,1,0.2);
    meeting=Collision.capsuleCapsule(origin,centre,x,1,0.02,origin,centre,slant,1,0.03);
    collinear=Collision.capsuleCapsule(origin,centre,z,1,0.02,origin,centre,z,1,0.03);
    assert(abs(nearlyParallel.gap-(d-0.04))<1e-12,"Capsules at a very small angle are apart by the distance of their centrelines less the radii");
    assert(abs(nearlyParallel.n[1])<1e-6 and abs(nearlyParallel.n[2])<1e-6 and abs(nearlyParallel.n[3]+1)<1e-6,
      "Their normal runs between the nearest points of the centrelines, here down from b to a");
    assert(abs(nearlyParallel.armA[1]-sqrt(1-e^2)/2)<1e-9 and abs(nearlyParallel.armA[2])<1e-9 and abs(nearlyParallel.armA[3]-0.02)<1e-9,
      "Their witness point on a is over the nearest point of a's centreline");
    assert(abs(onAxisZ.gap+0.3)<1e-12 and abs(onAxisX.gap+0.3)<1e-12 and abs(onAxisSlant.gap+0.3)<1e-12
      and abs(onAxisTilted.gap+0.3)<1e-12 and abs(atEnd.gap+0.3)<1e-12,
      "A sphere centred on a capsule's segment overlaps by the sum of the radii");
    assert(Tests.CollisionTests.offSurface(onAxisZ,centre,z,0.2)<1e-12 and Tests.CollisionTests.offSurface(onAxisX,centre,x,0.2)<1e-12
      and Tests.CollisionTests.offSurface(onAxisSlant,centre,slant,0.2)<1e-12
      and Tests.CollisionTests.offSurface(onAxisTilted,centre,tilted,0.2)<1e-12
      and Tests.CollisionTests.offSurface(atEnd,centre+z,z,0.2)<1e-12,
      "A sphere centred on a capsule's segment gets a unit normal across the capsule and a witness point on the capsule's surface");
    assert(abs(meeting.gap+0.05)<1e-12 and abs(collinear.gap+0.05)<1e-12,"Capsules whose centrelines meet overlap by the sum of the radii");
    assert(Tests.CollisionTests.offSurface(meeting,centre,slant,0.03)<1e-12 and abs(meeting.n*x)<1e-12
      and Tests.CollisionTests.offSurface(collinear,centre,z,0.03)<1e-12,
      "Capsules whose centrelines meet get a unit normal across both and witness points on their surfaces");
    for i in 1:3 loop
      assert(abs(onAxisZ.armA[i]-(centre[i]-0.1*onAxisZ.n[i]))<1e-12 and abs(meeting.armA[i]-(centre[i]-0.02*meeting.n[i]))<1e-12
        and abs(collinear.armA[i]-(centre[i]-0.02*collinear.n[i]))<1e-12,
        "The witness point on a is opposite, at a's own radius");
    end for;
  end CapsuleEdgeCases;

  model BodiesAgainstPairs "The shape tables of two bodies give the sum of their pairs' wrenches: balanced, the same in either order, mixed materials, and only for groups that may touch"
    import MultiBodyLie.Collision;
    import Kind=MultiBodyLie.Collision.ShapeKind;
    parameter Real r0A[3](each unit="m")={1024,-2048,512} "Frame of body a, far from the world origin";
    parameter Real offset[3](each unit="m")={0.25,0.125,-0.1875} "From a's frame to b's; binary fractions";
    parameter Real TA[3,3]=MultiBodyLie.Frames.axesRotationsT({1,2,3},{0.2,-0.3,0.5}) "World-to-frame rotation of a";
    parameter Real TB[3,3]=MultiBodyLie.Frames.axesRotationsT({3,1,2},{-0.4,0.1,0.3}) "World-to-frame rotation of b";
    parameter Real QA[3,3]=MultiBodyLie.Frames.axesRotationsT({2,3,1},{0.3,0.2,-0.1}) "From a's frame to its capsule's frame";
    parameter Real QBox[3,3]=MultiBodyLie.Frames.axesRotationsT({1,3,2},{-0.2,0.4,0.1}) "From b's frame to its box's frame";
    parameter Real QB[3,3]=MultiBodyLie.Frames.axesRotationsT({2,1,3},{0.5,-0.1,0.2}) "From b's frame to its capsule's frame";
    parameter Real axisA[3]=transpose(TA)*QA[1,:] "a's capsule, world axes";
    parameter Real axisB[3]=transpose(TB)*QB[1,:] "b's capsule, world axes";
    parameter Real TBox[3,3]=QBox*TB "World-to-box rotation";
    parameter Real across[3]=cross(axisA,axisB)/sqrt(cross(axisA,axisB)*cross(axisA,axisB)) "Unit vector across both capsules";
    parameter Real besideA[3]=cross(axisA,across) "Unit vector across a's capsule";
    parameter Real besideB[3]=cross(axisB,across) "Unit vector across b's capsule";
    // Every pair that may touch overlaps by 5 mm by construction. Each point is given in world axes relative
    // to the frame of its own body.
    parameter Real sphereA[3](each unit="m")={0.01,0.02,-0.01} "a's first sphere, radius 0.03";
    parameter Real sphereB[3](each unit="m")=sphereA+0.045*{2,-1,2}/3-offset "b's sphere, radius 0.02: 0.045 from a's first sphere";
    parameter Real boxB[3](each unit="m")=sphereA-transpose(TBox)*{0.075,0.01,0.005}-offset
      "b's box, half extents {0.05, 0.03, 0.02}: a's first sphere is 0.025 over its +x face";
    parameter Real capsuleA[3](each unit="m")=sphereB+offset-0.01*axisA-0.03*besideA
      "a's capsule, radius 0.015, half length 0.04: b's sphere is 0.03 beside its segment";
    parameter Real capsuleB[3](each unit="m")=capsuleA+0.03*across-offset
      "b's capsule, radius 0.02, half length 0.05: its segment crosses a's at 0.03";
    parameter Real sphereA2[3](each unit="m")=capsuleB+offset+0.02*axisB+0.03*besideB
      "a's second sphere, radius 0.015: 0.03 beside the segment of b's capsule";
    parameter Kind kindA[3]={Kind.Sphere,Kind.Capsule,Kind.Sphere};
    parameter Real rA[3,3](each unit="m")={TA*sphereA,TA*capsuleA,TA*sphereA2} "Shape origins in a's frame";
    parameter Real shapeTA[3,3,3]={identity(3),QA,identity(3)};
    parameter Real sizeA[3,3](each unit="m")={{0.03,0,0},{0.015,0.04,0},{0.015,0,0}};
    parameter Kind kindB[3]={Kind.Sphere,Kind.Box,Kind.Capsule};
    parameter Real rB[3,3](each unit="m")={TB*sphereB,TB*boxB,TB*capsuleB} "Shape origins in b's frame";
    parameter Real shapeTB[3,3,3]={identity(3),QBox,QB};
    parameter Real sizeB[3,3](each unit="m")={{0.02,0,0},{0.05,0.03,0.02},{0.02,0.05,0}};
    // Every shape has its own group. Five pairs may touch; a's capsule against the box (a pair without
    // geometry), a's first sphere against b's capsule and a's second sphere against b's sphere and box may not.
    parameter Boolean collides[6,6]={
      {false,false,false,true,true,false},
      {false,false,false,true,false,true},
      {false,false,false,false,false,true},
      {true,true,false,false,false,false},
      {true,false,false,false,false,false},
      {false,true,true,false,false,false}};
    parameter Real k[2](each unit="N/m")={1e4,3e4} "a's shapes are of material 1, b's of material 2";
    parameter Real d[2](each unit="s/m")={0.5,0.3};
    parameter Real mu[2]={0.8,0.5};
    parameter Real eps[2](each unit="m")={1e-4,2e-4};
    parameter Real v_eps[2](each unit="m/s")={0.01,0.02};
    parameter Collision.Material mixed(k=2e4,d=0.4,mu=0.8,eps=1.5e-4,v_eps=0.015) "Means of the two materials; the larger friction";
    parameter Real v0A[3](each unit="m/s")={0.03,-0.04,-0.05};
    parameter Real wA[3](each unit="rad/s")={0.7,-1.1,0.4} "Resolved in a's frame";
    parameter Real v0B[3](each unit="m/s")={-0.02,0.01,0.06};
    parameter Real wB[3](each unit="rad/s")={-0.3,0.9,0.2} "Resolved in b's frame";
    // The five pairs from the geometry functions, in the order of the tables.
    Collision.ContactPoint point1 "a's first sphere against b's sphere";
    Collision.ContactPoint point2 "a's first sphere against b's box";
    Collision.ContactPoint point3 "a's capsule against b's sphere";
    Collision.ContactPoint point4 "a's capsule against b's capsule";
    Collision.ContactPoint point5 "a's second sphere against b's capsule";
    parameter Collision.Material same=Collision.pairMaterial(mixed,mixed) "A material paired with itself";
    parameter Collision.Material extreme(k=1e308,d=1e-310,mu=0.8,eps=1e-4,v_eps=0.01)
      "A stiffness whose double is not a number of the machine, and a damping below the smallest normal number";
    parameter Collision.Material extremeSame=Collision.pairMaterial(extreme,extreme) "That material paired with itself";
    Real f1[3](each unit="N"),f2[3](each unit="N"),f3[3](each unit="N"),f4[3](each unit="N"),f5[3](each unit="N")
      "Force of each pair on a";
    Real t1[3](each unit="N.m"),t2[3](each unit="N.m"),t3[3](each unit="N.m"),t4[3](each unit="N.m"),t5[3](each unit="N.m")
      "Its torque on a";
    Real g1[3](each unit="N"),g2[3](each unit="N"),g3[3](each unit="N"),g4[3](each unit="N"),g5[3](each unit="N")
      "Force of each pair on b";
    Real s1[3](each unit="N.m"),s2[3](each unit="N.m"),s3[3](each unit="N.m"),s4[3](each unit="N.m"),s5[3](each unit="N.m")
      "Its torque on b";
    Real f[3](each unit="N") "Sum of the pairs' forces on a";
    Real t[3](each unit="N.m") "Sum of their torques on a";
    Real g[3](each unit="N") "Sum of their forces on b";
    Real s[3](each unit="N.m") "Sum of their torques on b";
    Real wAWorld[3](each unit="rad/s") "Angular velocity of a, world axes";
    Real wBWorld[3](each unit="rad/s") "Angular velocity of b, world axes";
    Real fA[3](each unit="N");
    Real tauA[3](each unit="N.m");
    Real fB[3](each unit="N");
    Real tauB[3](each unit="N.m");
    Real fAx[3](each unit="N") "Force on a with the two bodies exchanged";
    Real tauAx[3](each unit="N.m");
    Real fBx[3](each unit="N");
    Real tauBx[3](each unit="N.m");
    Real moment[3](each unit="N.m") "Total moment of the two wrenches about the origin of a's frame";
    Real scale(unit="N") "Size of the forces compared";
  equation
    moment=tauA+tauB+cross(offset,fB);
    wAWorld=transpose(TA)*wA;
    wBWorld=transpose(TB)*wB;
    point1=Collision.sphereSphere(r0A,sphereA,0.03,r0A+offset,sphereB,0.02);
    point2=Collision.sphereBox(r0A,sphereA,0.03,r0A+offset,boxB,TBox,{0.05,0.03,0.02});
    point3=Collision.flipped(Collision.sphereCapsule(r0A+offset,sphereB,0.02,r0A,capsuleA,axisA,0.04,0.015));
    point4=Collision.capsuleCapsule(r0A,capsuleA,axisA,0.04,0.015,r0A+offset,capsuleB,axisB,0.05,0.02);
    point5=Collision.sphereCapsule(r0A,sphereA2,0.015,r0A+offset,capsuleB,axisB,0.05,0.02);
    (f1,t1,g1,s1)=Collision.pairWrenches(point1,v0A,wAWorld,v0B,wBWorld,mixed);
    (f2,t2,g2,s2)=Collision.pairWrenches(point2,v0A,wAWorld,v0B,wBWorld,mixed);
    (f3,t3,g3,s3)=Collision.pairWrenches(point3,v0A,wAWorld,v0B,wBWorld,mixed);
    (f4,t4,g4,s4)=Collision.pairWrenches(point4,v0A,wAWorld,v0B,wBWorld,mixed);
    (f5,t5,g5,s5)=Collision.pairWrenches(point5,v0A,wAWorld,v0B,wBWorld,mixed);
    f=f1+f2+f3+f4+f5;
    t=t1+t2+t3+t4+t5;
    g=g1+g2+g3+g4+g5;
    s=s1+s2+s3+s4+s5;
    assert(abs(point1.gap+0.005)<1e-12 and abs(point2.gap+0.005)<1e-12 and abs(point3.gap+0.005)<1e-12
      and abs(point4.gap+0.005)<1e-12 and abs(point5.gap+0.005)<1e-12,"Every pair of the fixture overlaps by 5 mm");
    assert(f1*point1.n>1 and f2*point2.n>1 and f3*point3.n>1 and f4*point4.n>1 and f5*point5.n>1,
      "Every pair of the fixture presses");
    (fA,tauA,fB,tauB)=Collision.bodiesWrench(r0A,TA,wA,v0A,kindA,rA,shapeTA,sizeA,ones(3),{1,2,3},
      r0A+offset,TB,wB,v0B,kindB,rB,shapeTB,sizeB,fill(2,3),{4,5,6},k,d,mu,eps,v_eps,collides);
    (fBx,tauBx,fAx,tauAx)=Collision.bodiesWrench(r0A+offset,TB,wB,v0B,kindB,rB,shapeTB,sizeB,fill(2,3),{4,5,6},
      r0A,TA,wA,v0A,kindA,rA,shapeTA,sizeA,ones(3),{1,2,3},k,d,mu,eps,v_eps,collides);
    assert(abs(same.k-mixed.k)<=0 and abs(same.d-mixed.d)<=0 and abs(same.mu-mixed.mu)<=0 and abs(same.eps-mixed.eps)<=0
      and abs(same.v_eps-mixed.v_eps)<=0,"Two shapes of one material have exactly that material");
    assert(abs(extremeSame.k-extreme.k)<=0 and abs(extremeSame.d-extreme.d)<=0 and abs(extremeSame.eps-extreme.eps)<=0
      and abs(extremeSame.v_eps-extreme.v_eps)<=0,"Two shapes of one material have exactly that material at the ends of the number range too");
    assert(abs(Collision.midpoint(1e308,-1e308))<=0 and abs(Collision.midpoint(3e307,1e308)-6.5e307)<1e293
      and abs(Collision.midpoint(1e308,3e307)-Collision.midpoint(3e307,1e308))<=0 and abs(Collision.midpoint(-2,-4)+3)<=0
      and abs(Collision.midpoint(-1,3)-1)<=0,
      "The mean of two numbers is finite for large numbers of either sign and the same in either order");
    scale=sqrt(f1*f1+f2*f2+f3*f3+f4*f4+f5*f5);
    for i in 1:3 loop
      assert(abs(fA[i]-f[i])<1e-9*scale and abs(tauA[i]-t[i])<1e-9*scale
        and abs(fB[i]-g[i])<1e-9*scale and abs(tauB[i]-s[i])<1e-9*scale,
        "The wrenches of two bodies are the sums of the wrenches of the pairs of shapes that may touch");
      assert(abs(fA[i]+fB[i])<=0,"The forces on the two bodies are exactly opposite");
      assert(abs(moment[i])<1e-9*scale,
        "The wrenches on the two bodies have no total moment");
      assert(abs(fAx[i]-fA[i])<1e-9*scale and abs(tauAx[i]-tauA[i])<1e-9*scale and abs(fBx[i]-fB[i])<1e-9*scale
        and abs(tauBx[i]-tauB[i])<1e-9*scale,"Exchanging the two bodies exchanges their wrenches");
    end for;
  end BodiesAgainstPairs;

  model CollidingBodies "Two free bodies with a BodyPair meet off centre: their momentum and angular momentum stay, and they part"
    import MultiBodyLie.Collision;
    parameter Real mA(unit="kg")=0.2;
    parameter Real mB(unit="kg")=0.3;
    parameter Real IA[3](each unit="kg.m2")={1e-4,2e-4,1.5e-4} "Principal inertias of a about its origin";
    parameter Real IB[3](each unit="kg.m2")={2e-4,1e-4,3e-4} "Principal inertias of b about its origin";
    parameter Real vA0[3](each unit="m/s")={0.5,0,0} "a moves towards b at the start";
    parameter Real wB0[3](each unit="rad/s")={0.3,-0.2,0.1} "b spins at the start";
    inner MultiBodyLie.World world(g=0,enableAnimation=false);
    inner Collision.Scene scene(k={1e4});
    MultiBodyLie.Parts.Body a(m=mA,I_11=IA[1],I_22=IA[2],I_33=IA[3],
      r_0(start={0,0,0},each fixed=true),v_0(start=vA0,each fixed=true),
      angles_fixed=true,w_0_fixed=true,w_0_start={0,0,0});
    MultiBodyLie.Parts.Body b(m=mB,I_11=IB[1],I_22=IB[2],I_33=IB[3],
      r_0(start={0.1,0.02,0.01},each fixed=true),v_0(start={0,0,0},each fixed=true),
      angles_fixed=true,w_0_fixed=true,w_0_start=wB0);
    Collision.BodyPair pair(nA=1,sizeA={{0.03,0,0}},nB=1,sizeB={{0.02,0,0}}) "A sphere of 30 mm against a sphere of 20 mm";
    Real p[3](each unit="kg.m/s") "Total momentum";
    Real L[3](each unit="kg.m2/s") "Total angular momentum about the world origin";
  equation
    connect(a.frame_a,pair.frame_a);
    connect(b.frame_a,pair.frame_b);
    pair.v0A=a.v_0;
    pair.v0B=b.v_0;
    p=mA*a.v_0+mB*b.v_0;
    L=mA*cross(a.r_0,a.v_0)+transpose(a.frame_a.R.T)*(IA.*a.frame_a.R.w)
      +mB*cross(b.r_0,b.v_0)+transpose(b.frame_a.R.T)*(IB.*b.frame_a.R.w);
    // a reaches b after about 0.11 s; the contact lasts about 0.01 s.
    assert(time<0.19 or (b.v_0[1]>0.1 and (b.r_0-a.r_0)*(b.v_0-a.v_0)>0),"The bodies must have met and be parting at the end of the run");
    for i in 1:3 loop
      assert(abs(p[i]-mA*vA0[i])<1e-8,"Contact between two bodies must not change their total momentum");
      assert(abs(L[i]-IB[i]*wB0[i])<1e-6,"Contact between two bodies must not change their total angular momentum");
    end for;
  end CollidingBodies;

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

  model Suite "Plane pairs and the scene tables against the ground wrench; sphere and capsule pairs against analytic cases and at their degenerate poses; two bodies against their pairs; a falling body on both contact models and two colliding bodies"
    PlaneSphereAgainstGroundWrench planeSphere;
    SoleAgainstGroundWrench sole;
    CapsuleAndBoxAgainstGroundWrench capsuleAndBox;
    FallingBodies falling;
    SpherePairs spherePairs;
    CapsulePairs capsulePairs;
    CapsuleEdgeCases capsuleEdges;
    BodiesAgainstPairs bodies;
    CollidingBodies colliding;
    SceneTables tables;
    annotation(experiment(StartTime=0,StopTime=0.2,Interval=0.002,Tolerance=1e-8));
  end Suite;
end CollisionTests;
