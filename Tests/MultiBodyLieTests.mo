within Tests;
package MultiBodyLieTests "Gravity field, orientation residue, tree dynamics and ground contact of MultiBodyLie"
  model FreeFall "A free body's centre of mass accelerates with the world's gravity field there"
    parameter MultiBodyLie.Types.GravityTypes gravityType=MultiBodyLie.Types.GravityTypes.UniformGravity
      annotation(Evaluate=true);
    parameter Real n[3]={0,-1,0};
    parameter Real mu=3.986004418e14;
    parameter Real r_start[3]=zeros(3);
    parameter Real r_CM[3]={0.1,-0.05,0.02};
    inner MultiBodyLie.World world(gravityType=gravityType,n=n,mu=mu,enableAnimation=false);
    MultiBodyLie.Parts.Body body(m=2,r_CM=r_CM,I_11=0.04,I_22=0.05,I_33=0.06,I_21=0.002,
      r_0(start=r_start,each fixed=true),v_0(start={0.1,0.2,-0.3},each fixed=true),
      angles_fixed=true,w_0_fixed=true,w_0_start={0.3,-0.2,0.1});
    Real centre[3](each unit="m")=body.r_0+transpose(body.frame_a.R.T)*r_CM "Centre of mass, world";
    Real expected[3](each unit="m/s2") "Field at the centre of mass, world";
    Real centreAcceleration[3](each unit="m/s2") "Acceleration of the centre of mass, world";
  equation
    expected=if gravityType==MultiBodyLie.Types.GravityTypes.UniformGravity then world.g*n/sqrt(n*n)
      elseif gravityType==MultiBodyLie.Types.GravityTypes.PointGravity then -mu/(centre*centre)*centre/sqrt(centre*centre)
      else zeros(3);
    centreAcceleration=body.a_0+transpose(body.frame_a.R.T)*(cross(body.frame_a.a[1:3],r_CM)
      +cross(body.w_a,cross(body.w_a,r_CM)));
    for i in 1:3 loop
      assert(abs(body.g_0[i]-expected[i])<1e-9*max(1,abs(expected[i])),"Body gravity differs from the world field");
      assert(abs(centreAcceleration[i]-expected[i])<1e-6*max(1,abs(expected[i])),"Centre of mass does not fall with the field");
    end for;
  end FreeFall;

  model HalfTurnResidue "The orientation residue is the relative rotation vector, nonzero for a half turn"
    parameter Real halfTurn[3,3]=diagonal({-1,-1,1}) "Half turn about z";
    parameter Real small[3]={0.01,-0.02,0.015};
    Real halfTurnResidue[3]=MultiBodyLie.Frames.orientationResidue(halfTurn,identity(3));
    Real smallResidue[3]=MultiBodyLie.Frames.orientationResidue(
      transpose(LieGroups.SO3.Dcm.exp_map(small)),identity(3));
  equation
    assert(abs(sqrt(halfTurnResidue*halfTurnResidue)-Modelica.Constants.pi)<1e-9,
      "A half turn must leave a residue of length pi");
    for i in 1:3 loop
      // T maps world to frame, so the frame's rotation by small leaves the residue -small.
      assert(abs(smallResidue[i]+small[i])<1e-12,"Residue must be the relative rotation vector");
    end for;
  end HalfTurnResidue;

  model FloatingTree "Math.floatingTreeDynamics agrees with the articulated-body components"
    parameter Real g=9.80665;
    parameter Real n[3]={0.2,-1,0.3};
    Tests.RevoluteArmatureTests.Chain chain(folded=true,world(g=g,n=n));
    Real massMatrix[8,8];
    Real bias[8];
    Real acceleration[8] "Root body {dw; dv}, then joint accelerations";
    Real rootAcceleration[3](each unit="m/s2") "World acceleration of the root origin from the tree solve";
  equation
    (massMatrix,bias,acceleration)=MultiBodyLie.Math.floatingTreeDynamics(
      {0,1,2},
      {zeros(3),chain.link.r},
      {identity(3),identity(3)},
      {chain.first.n,chain.second.n},
      {chain.first.phi,chain.second.phi},
      cat(1,chain.base.w_a,chain.base.frame_a.v,{chain.first.w,chain.second.w}),
      {MultiBodyLie.Math.spatialInertia(chain.base.m,chain.base.r_CM,
         {{chain.base.I_11,chain.base.I_21,chain.base.I_31},{chain.base.I_21,chain.base.I_22,chain.base.I_32},{chain.base.I_31,chain.base.I_32,chain.base.I_33}}),
       MultiBodyLie.Math.spatialInertia(chain.link.m,chain.link.r_CM,
         {{chain.link.I_11,chain.link.I_21,chain.link.I_31},{chain.link.I_21,chain.link.I_22,chain.link.I_32},{chain.link.I_31,chain.link.I_32,chain.link.I_33}}),
       MultiBodyLie.Math.spatialInertia(chain.tip.m,chain.tip.r_CM,
         {{chain.tip.I_11,chain.tip.I_21,chain.tip.I_31},{chain.tip.I_21,chain.tip.I_22,chain.tip.I_32},{chain.tip.I_31,chain.tip.I_32,chain.tip.I_33}})},
      {chain.first.armature,chain.second.armature},
      chain.base.frame_a.R.T*(g*n/sqrt(n*n)),
      cat(1,zeros(6),{chain.drive1.command,chain.drive2.command}));
    rootAcceleration=transpose(chain.base.frame_a.R.T)*(acceleration[4:6]+cross(chain.base.w_a,chain.base.frame_a.v));
    assert(abs(acceleration[7]-chain.first.a)<1e-8,"First joint acceleration differs from the component pass");
    assert(abs(acceleration[8]-chain.second.a)<1e-8,"Second joint acceleration differs from the component pass");
    for i in 1:3 loop
      assert(abs(acceleration[i]-chain.base.frame_a.a[i])<1e-8,"Root angular acceleration differs from the component pass");
      assert(abs(rootAcceleration[i]-chain.base.a_0[i])<1e-8,"Root linear acceleration differs from the component pass");
    end for;
  end FloatingTree;

  model GroundSphere "Hunt-Crossley normal force, regularized friction and torque of one sphere"
    parameter Real k=1e4;
    parameter Real d=0.5;
    parameter Real mu=0.8;
    parameter Real eps=1e-4;
    parameter Real v_eps=0.01;
    parameter Real radius=0.01;
    parameter Real depth=0.002 "Penetration of the sphere's lowest point";
    parameter Real slip[2]={0.03,-0.04} "Sliding velocity of the frame origin, world";
    parameter Real sink=-0.05 "Vertical velocity, world (negative = moving into the ground)";
    parameter Real centre[1,3]={{0.02,0,0}} "Sphere centre in frame_a";
    Real f[3];
    Real tau[3];
    Real fClear[3];
    Real tauClear[3];
    Real fn "Expected normal force";
    Real expectedTorque[3] "Expected force at the lowest point, moment about the origin";
    Real vt "Expected regularized slip speed";
  equation
    // frame_a level with the world, origin chosen so the sphere bottom is depth below z = 0.
    (f,tau)=MultiBodyLie.Contact.groundWrench({0,0,radius-depth},identity(3),zeros(3),
      {slip[1],slip[2],sink},centre,{radius},k,d,mu,eps,v_eps);
    (fClear,tauClear)=MultiBodyLie.Contact.groundWrench({0,0,1},identity(3),zeros(3),
      {slip[1],slip[2],sink},centre,{radius},k,d,mu,eps,v_eps);
    fn=k*MultiBodyLie.Contact.contactSoftplus(depth,eps)*MultiBodyLie.Contact.contactSoftplus(1-d*sink,0.05);
    vt=sqrt(slip*slip+v_eps^2);
    expectedTorque=cross({centre[1,1],centre[1,2],centre[1,3]-radius},f);
    assert(abs(f[3]-fn)<1e-9*fn,"Normal force must follow the smooth Hunt-Crossley law");
    assert(abs(f[1]+mu*fn*slip[1]/vt)<1e-9*fn and abs(f[2]+mu*fn*slip[2]/vt)<1e-9*fn,
      "Friction must oppose the slip with regularized Coulomb magnitude");
    for i in 1:3 loop
      assert(abs(tau[i]-expectedTorque[i])<1e-12,"Torque must be the contact force at the sphere's lowest point");
      assert(abs(fClear[i])+abs(tauClear[i])<1e-300,"A sphere clear of the ground carries no force");
    end for;
  end GroundSphere;

  model Suite "Uniform, point and absent gravity; residue; tree dynamics; contact"
    FreeFall uniform;
    FreeFall uniformDown(n={0,0,-1});
    FreeFall point(gravityType=MultiBodyLie.Types.GravityTypes.PointGravity,r_start={7.0e6,1.0e5,-2.0e5});
    FreeFall none(gravityType=MultiBodyLie.Types.GravityTypes.NoGravity);
    HalfTurnResidue residue;
    FloatingTree tree;
    GroundSphere sphere;
    annotation(experiment(StartTime=0,StopTime=0.1,Interval=0.002,Tolerance=1e-8));
  end Suite;
end MultiBodyLieTests;
