within Tests;
package RevoluteArmatureTests "Joint-owned armature versus the original external rotor law"
  model ExternalRotor "Reference law used by Microduck before the armature fold"
    parameter Real J(unit="kg.m2")=0.0018;
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange;
    input Real command(unit="N.m");
    Real velocity(unit="rad/s");
  equation
    velocity=der(flange.phi);
    flange.tau=J*der(velocity)-command;
  end ExternalRotor;

  model CommandTorque "Command port with no reflected inertia in the actuator"
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange;
    input Real command(unit="N.m");
  equation
    flange.tau=-command;
  end CommandTorque;

  model Pair "Compare positions, shaft torque, wrench and rotor power in either tree direction"
    parameter Real J(unit="kg.m2")=0.0018;
    parameter Boolean reverse=false;
    parameter Real axis[3]={0,0,1};
    inner MultiBodyLie.World world(g=0,enableAnimation=false);
    MultiBodyLie.Joints.Revolute reference(n=axis,useAxisFlange=true,
      phi(start=0.2,fixed=true),w(start=0.3,fixed=true));
    MultiBodyLie.Joints.Revolute folded(n=axis,useAxisFlange=true,armature=J,
      phi(start=0.2,fixed=true),w(start=0.3,fixed=true));
    MultiBodyLie.Parts.Body referenceBody(m=1,I_11=0.02,I_22=0.02,I_33=0.02);
    MultiBodyLie.Parts.Body foldedBody(m=1,I_11=0.02,I_22=0.02,I_33=0.02);
    ExternalRotor rotor(J=J);
    CommandTorque actuator;
    Real command(unit="N.m")=0.03+0.02*sin(3*time);
    Real rotorEnergy(unit="J", start=J*0.3*0.3/2, fixed=true);
    Real rotorPower(unit="W")=folded.axis.tau*folded.w-folded.tau*folded.w;
  equation
    if reverse then
      connect(world.frame_b,reference.frame_b);
      connect(reference.frame_a,referenceBody.frame_a);
      connect(world.frame_b,folded.frame_b);
      connect(folded.frame_a,foldedBody.frame_a);
    else
      connect(world.frame_b,reference.frame_a);
      connect(reference.frame_b,referenceBody.frame_a);
      connect(world.frame_b,folded.frame_a);
      connect(folded.frame_b,foldedBody.frame_a);
    end if;
    connect(rotor.flange,reference.axis);
    connect(actuator.flange,folded.axis);
    rotor.command=command;
    actuator.command=command;
    assert(abs(reference.phi-folded.phi)<1e-8,"armature position mismatch");
    assert(abs(reference.w-folded.w)<1e-8,"armature velocity mismatch");
    assert(abs(reference.a-folded.a)<1e-8,"armature acceleration mismatch");
    assert(abs(reference.tau-folded.tau)<1e-8,"net shaft torque mismatch");
    assert(abs(folded.a-command/(0.02+J))<1e-8,"armature analytic acceleration mismatch");
    assert(abs(rotor.flange.tau+folded.tau)<1e-8,"old shaft reaction mapping mismatch");
    der(rotorEnergy)=rotorPower;
    assert(abs(rotorEnergy-J*folded.w*folded.w/2)<1e-8,"rotor energy balance mismatch");
    for i in 1:3 loop
      assert(abs(reference.frame_a.f[i]-folded.frame_a.f[i])<1e-8,"parent force mismatch");
      assert(abs(reference.frame_a.t[i]-folded.frame_a.t[i])<1e-8,"parent moment mismatch");
      assert(abs(reference.frame_b.f[i]-folded.frame_b.f[i])<1e-8,"child force mismatch");
      assert(abs(reference.frame_b.t[i]-folded.frame_b.t[i])<1e-8,"child moment mismatch");
    end for;
  end Pair;

  model Chain "Two joints on a freely moving base exercise the inward inertia elimination"
    parameter Boolean folded=false;
    parameter Real J1(unit="kg.m2")=0.0018;
    parameter Real J2(unit="kg.m2")=0.05;
    inner MultiBodyLie.World world(g=0,enableAnimation=false);
    MultiBodyLie.Parts.Body base(m=2,I_11=0.04,I_22=0.05,I_33=0.06,
      r_0(start={0,0,0},each fixed=true),v_0(start={0.1,-0.05,0.02},each fixed=true),
      angles_fixed=true,w_0_fixed=true,w_0_start={0.2,-0.1,0.3});
    MultiBodyLie.Joints.Revolute first(n={1,1,0},armature=if folded then J1 else 0,
      phi(start=0.2,fixed=true),w(start=0.3,fixed=true));
    MultiBodyLie.Joints.Revolute second(n={0,1,1},armature=if folded then J2 else 0,
      phi(start=-0.1,fixed=true),w(start=-0.2,fixed=true));
    MultiBodyLie.Parts.BodyShape link(r={0.1,0.02,-0.05},r_CM={0.04,0.01,-0.02},m=0.5,
      I_11=0.012,I_22=0.015,I_33=0.011,I_21=0.001);
    MultiBodyLie.Parts.Body tip(r_CM={0.01,0.02,0.03},m=0.3,
      I_11=0.009,I_22=0.008,I_33=0.007);
    ExternalRotor drive1(J=if folded then 0 else J1);
    ExternalRotor drive2(J=if folded then 0 else J2);
  equation
    Connections.root(base.frame_a.R);
    connect(base.frame_a,first.frame_a);
    connect(first.frame_b,link.frame_a);
    connect(link.frame_b,second.frame_a);
    connect(second.frame_b,tip.frame_a);
    connect(drive1.flange,first.axis);
    connect(drive2.flange,second.axis);
    drive1.command=0.03+0.02*sin(3*time);
    drive2.command=-0.02+0.01*cos(2*time);
  end Chain;

  model FreePair "Moving-parent and coupled-body regression for the complete Schur complement"
    Chain reference;
    Chain folded(folded=true);
  equation
    assert(abs(reference.first.a-folded.first.a)<1e-7,"moving-parent first acceleration mismatch");
    assert(abs(reference.second.a-folded.second.a)<1e-7,"moving-parent second acceleration mismatch");
    assert(abs(reference.first.tau-folded.first.tau)<1e-8,"moving-parent shaft mismatch");
    assert(abs(reference.second.tau-folded.second.tau)<1e-8,"moving-child shaft mismatch");
    for i in 1:3 loop
      assert(abs(reference.base.r_0[i]-folded.base.r_0[i])<1e-8,"free base position mismatch");
      assert(abs(reference.base.a_0[i]-folded.base.a_0[i])<1e-7,"free base acceleration mismatch");
      assert(abs(reference.base.w_a[i]-folded.base.w_a[i])<1e-8,"free base rotation mismatch");
      assert(abs(reference.first.frame_a.f[i]-folded.first.frame_a.f[i])<1e-7,"free base force mismatch");
      assert(abs(reference.first.frame_a.t[i]-folded.first.frame_a.t[i])<1e-7,"free base moment mismatch");
      assert(abs(reference.second.frame_a.f[i]-folded.second.frame_a.f[i])<1e-7,"moving-parent force mismatch");
      assert(abs(reference.second.frame_a.t[i]-folded.second.frame_a.t[i])<1e-7,"moving-parent moment mismatch");
    end for;
  end FreePair;

  model InvalidArmature "A negative reflected inertia must be refused"
    extends Pair(J=-0.001);
  end InvalidArmature;

  model Suite "Zero, Microduck and larger armature, forward and reverse trees"
    FreePair freeMovingBase;
    Pair zero(J=0);
    Pair microduck(J=0.0018);
    Pair larger(J=0.05);
    Pair reverseZero(J=0,reverse=true);
    Pair reverseMicroduck(J=0.0018,reverse=true);
    Pair reverseLarger(J=0.05,reverse=true,axis={0,0,-1});
    annotation(experiment(StartTime=0,StopTime=0.1,Interval=0.002,Tolerance=1e-8));
  end Suite;
end RevoluteArmatureTests;
