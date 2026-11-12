within Vehicles.Microduck;
package Mechanics "Drive, mass-property and rest-pose helpers for the Microduck assembly"
  record MassProperties
    Real m;
    Real c[3];
    Real I[3,3];
  end MassProperties;
  model Drive "Commanded shaft torque at a servo axis; the reflected rotor inertia is the joint armature"
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange;
    input Real command;
    Real velocity;
  equation
    velocity = der(flange.phi);
    flange.tau = -command;
  end Drive;

  function tensor
    input Real six[6] "xx,yy,zz,xy,xz,yz";
    output Real value[3,3];
  algorithm
    value := {{six[1],six[4],six[5]},{six[4],six[2],six[6]},{six[5],six[6],six[3]}};
  end tensor;

  function parallelAxis
    input Real mass;
    input Real centre[3];
    output Real value[3,3];
  algorithm
    value := mass*((centre*centre)*identity(3)-{{centre[i]*centre[j] for j in 1:3} for i in 1:3});
  end parallelAxis;

  function splitHead "Remove the estimated closed jaw from the imported lumped head"
    input Real totalMass;
    input Real totalCentre[3];
    input Real totalInertia[6];
    input Real jawMass;
    input Real jawCentreInHead[3];
    input Real jawToHead[3,3];
    input Real jawInertia[3,3];
    output MassProperties head;
  algorithm
    head.m := totalMass-jawMass;
    assert(head.m > 0,"Jaw mass must be smaller than lumped head mass");
    head.c := (totalMass*totalCentre-jawMass*jawCentreInHead)/head.m;
    head.I := tensor(totalInertia)+parallelAxis(totalMass,totalCentre)
      -jawToHead*jawInertia*transpose(jawToHead)-parallelAxis(jawMass,jawCentreInHead)
      -parallelAxis(head.m,head.c);
    assert(head.I[1,1] > 0 and head.I[1,1]*head.I[2,2]-head.I[1,2]^2 > 0,
      "Estimated jaw split makes head inertia nonphysical");
    assert(head.I[1,1]*head.I[2,2]*head.I[3,3]
      +2*head.I[1,2]*head.I[1,3]*head.I[2,3]
      -head.I[1,1]*head.I[2,3]^2-head.I[2,2]*head.I[1,3]^2
      -head.I[3,3]*head.I[1,2]^2 > 0,"Head inertia must be positive definite");
  end splitHead;

  function scaleMass "Uniform segment scaling at fixed density"
    input Real mass[:];
    input Real scale[size(mass,1)];
    output Real value[size(mass,1)];
  algorithm
    for i in 1:size(mass,1) loop
      assert(scale[i] > 0,"Physical scale must be positive");
      value[i] := mass[i]*scale[i]^3;
    end for;
  end scaleMass;

  function scaleInertia
    input Real inertia[:,6];
    input Real scale[size(inertia,1)];
    output Real value[size(inertia,1),6];
  algorithm
    for i in 1:size(inertia,1) loop
      for j in 1:6 loop
        value[i,j] := inertia[i,j]*scale[i]^5;
      end for;
    end for;
  end scaleInertia;

  function restTree "Parent-first FK; root origin and orientation are identity"
    input Integer parent[:];
    input Real offset[size(parent,1),3];
    input Real quaternion[size(parent,1),4];
    input Real axis[size(parent,1)-1,3];
    input Real angle[size(parent,1)-1];
    output Real position[size(parent,1),3];
    output Real toWorld[size(parent,1),3,3];
  protected
    Integer p;
  algorithm
    position := zeros(size(parent,1),3);
    toWorld := zeros(size(parent,1),3,3);
    position[1,:] := zeros(3);
    toWorld[1,:,:] := identity(3);
    for i in 2:size(parent,1) loop
      p := parent[i];
      assert(p > 0 and p < i,"Body data must be a parent-first tree");
      position[i,:] := position[p,:]+toWorld[p,:,:]*offset[i,:];
      toWorld[i,:,:] := toWorld[p,:,:]*transpose(transpose(LieGroups.SO3.Quat.to_DCM(quaternion[i,:])))
        *LieGroups.SO3.Dcm.exp_map(axis[i-1,:]*angle[i-1]);
    end for;
  end restTree;

  function initialHeight
    input Integer parent[:];
    input Real offset[size(parent,1),3];
    input Real quaternion[size(parent,1),4];
    input Real axis[size(parent,1)-1,3];
    input Real angle[size(parent,1)-1];
    input Integer footBodies[2];
    input Real footOffsets[2,3];
    input Real footQuaternions[2,4];
    input Real halfLength;
    input Real halfWidth;
    input Real radius;
    output Real height;
  protected
    Real p[size(parent,1),3];
    Real R[size(parent,1),3,3];
    Real point[3];
    Real corner[3];
    Real minimum;
    Integer body;
  algorithm
    (p,R) := restTree(parent,offset,quaternion,axis,angle);
    minimum := 1e6;
    for foot in 1:2 loop
      body := footBodies[foot];
      for x in 1:2 loop
        for y in 1:2 loop
          corner := {(if x==1 then -1 else 1)*halfLength,(if y==1 then -1 else 1)*halfWidth,0};
          point := p[body,:]+R[body,:,:]*(footOffsets[foot,:]
            +transpose(transpose(LieGroups.SO3.Quat.to_DCM(footQuaternions[foot,:])))*corner);
          minimum := min(minimum,point[3]-radius);
        end for;
      end for;
    end for;
    height := -minimum+0.001;
  end initialHeight;
end Mechanics;
