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
      assert(abs(f[i]-fGround[i])<=0,"The scene tables must give the ground wrench's force bit for bit");
      assert(abs(tau[i]-tauGround[i])<=0,"The scene tables must give the ground wrench's torque bit for bit");
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

  model Suite "Plane pairs and the scene tables against the ground wrench; sphere and capsule pairs against analytic cases; a falling body on both contact models"
    PlaneSphereAgainstGroundWrench planeSphere;
    SoleAgainstGroundWrench sole;
    CapsuleAndBoxAgainstGroundWrench capsuleAndBox;
    FallingBodies falling;
    SpherePairs spherePairs;
    CapsulePairs capsulePairs;
    SceneTables tables;
    annotation(experiment(StartTime=0,StopTime=0.2,Interval=0.002,Tolerance=1e-8));
  end Suite;
end CollisionTests;
