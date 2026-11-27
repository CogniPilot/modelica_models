within Tests;
model MarinePhysicsTests "Check Fourier surface sampling, hydrostatic current and contact conventions"
  function checkDvl
    output Boolean passed;
  protected
    constant Real phi=Modelica.Constants.pi/8;
    constant Real axes[4,3]=[sin(phi),0,-cos(phi);-sin(phi),0,-cos(phi);
      0,sin(phi),-cos(phi);0,-sin(phi),-cos(phi)];
    Real reference[4,3],observed[4,3],dz;
    Boolean valid;
  algorithm
    reference:=Marine.Wreck.Perception.dvlWorldPoints({2,3,-4},0,0,Modelica.Constants.pi/2,
      fill(2,4),axes);
    assert(abs(reference[1,2]-3-2*sin(phi))<1e-9 and
      abs(reference[1,3]+4+.15+2*cos(phi))<1e-9,
      "DVL beam uses the transducer offset, slant range and ENU attitude");
    observed:=reference;observed[:,3]:=observed[:,3]+fill(.10,4);
    (dz,valid):=Marine.Wreck.Perception.dvlHeightCorrection(observed,reference);
    assert(valid and abs(dz+.10)<1e-9,"Overlapping coplanar returns correct positive height bias downward");
    observed[1,3]:=observed[1,3]+.30;
    (dz,valid):=Marine.Wreck.Perception.dvlHeightCorrection(observed,reference);
    assert(not valid,"A discontinuous DVL footprint cannot constrain height");
    observed:=reference;observed[:,1]:=observed[:,1]+fill(5.,4);
    (dz,valid):=Marine.Wreck.Perception.dvlHeightCorrection(observed,reference);
    assert(not valid,"A spatially disjoint footprint cannot constrain height");
    passed:=true;
  end checkDvl;
  function checkStayPanelContact "Wire clearance, action/reaction and moment at an interior panel"
    output Boolean passed;
  protected
    constant Real cloth[8,3]=[0,0,0;1,0,0;2,0,0;3,0,0;0,0,1;1,0,1;2,0,1;3,0,1];
    constant Real folded[8,3]=[0,0,0;1,0,1;2,0,1;3,0,1;1,0,-1;0,1,0;2,1,0;3,1,0];
    Real wire[3,3],clothForce[8,3],wireForce[3,3],moment[3];
    Integer side;
  algorithm
    for trial in 1:2 loop
      side:=2*trial-3;
      wire:=[.5,side*.1,.24;.5,side*.002,.25;.5,side*.1,.26];
      (clothForce,wireForce):=Marine.Sailing.stayPanelContact(wire,cloth);
      assert(side*sum(wireForce[:,2])>1,"The interior of a panel repels the nearby wire on either tack");
      moment:=zeros(3);
      for i in 1:8 loop moment:=moment+cross(cloth[i,:],clothForce[i,:]); end for;
      for i in 1:3 loop moment:=moment+cross(wire[i,:],wireForce[i,:]); end for;
      for k in 1:3 loop
        assert(abs(sum(clothForce[:,k])+sum(wireForce[:,k]))<1e-9,"Contact preserves total force");
      end for;
      assert(max(abs(moment))<1e-9,"Contact preserves total moment");
    end for;
    wire[:,3]:=wire[:,3]+fill(2,3);
    (clothForce,wireForce):=Marine.Sailing.stayPanelContact(wire,cloth);
    assert(max(abs(clothForce))+max(abs(wireForce))<1e-10,"A finite stay beyond the sail has no contact");
    // The first half intersects both faces of a fold. Roundoff in the line-plane
    // coordinates must not send the reaction to the opposite side of the cable.
    for trial in 0:1 loop
      wire:=[.2,.2,-3.368;.2,.2,2.88+trial*1e-12;.2,.2,3.88+trial*1e-12];
      (clothForce,wireForce):=Marine.Sailing.stayPanelContact(wire,folded);
      assert(sum(wireForce[:,1])>2.8 and sum(wireForce[:,3])< -2.8,
        "Coincident face crossings keep the first face's normal");
    end for;
    passed:=true;
  end checkStayPanelContact;
  function checkEchoPacking "Verify independent bearing rows and near-to-far byte order"
    output Boolean passed;
  protected
    constant Real t=0.25;
    constant Integer hitCell=45;
    constant Real hitRange=(hitCell-0.5)*Marine.Wreck.ImagingSonar.rangeMax/Marine.Wreck.ImagingSonar.bins;
    Real ranges[Marine.Wreck.ImagingSonar.bearings,Marine.Wreck.ImagingSonar.verticalSamples], words[Marine.Wreck.ImagingSonar.bearings,32], profile[49];
    Real expected;
  algorithm
    ranges:=fill(Marine.Wreck.ImagingSonar.rangeMax,Marine.Wreck.ImagingSonar.bearings,Marine.Wreck.ImagingSonar.verticalSamples);
    ranges[1,:]:=fill(hitRange,Marine.Wreck.ImagingSonar.verticalSamples);
    (words,profile):=Marine.Wreck.ImagingSonar.formEcho(t,zeros(3),identity(3),ranges);
    assert(abs(profile[1]-hitRange)<1e-10 and max(abs(profile[2:49]))<1e-10,
      "Only the first bearing sees the reflector, at the center of its range cell");
    assert(mod(words[1,div(hitCell-1,4)+1],256)>100,
      "The near byte of the hit word contains the reflector");
    for bearing in 2:Marine.Wreck.ImagingSonar.bearings loop
      for w in 1:32 loop
        expected:=sum(256.0^(k-1)*floor(2*(1+Marine.Wreck.Perception.noise(t,4*(w-1)+k+3*bearing))) for k in 1:4);
        assert(abs(words[bearing,w]-expected)<0.5,"Each empty bearing retains its four noise bytes in range order");
      end for;
    end for;
    passed:=true;
  end checkEchoPacking;
  function check
    output Boolean passed;
  protected
    constant Real station[8,3]={{0.0,0.0,1.0*i} for i in 1:8};
    Real surface[32,32],probe[5],slope[2],eastPlus[5],eastMinus[5],dummy[32,32],ds[2];
    Real currentENU[3],plane[4],contact[7];
  algorithm
    (surface,probe,slope):=Marine.Environment.SpectralSea.field(8,11,.4,45000,13.7,0,{8,4,0},.45);
    assert(abs(surface[3,5]-probe[1])<1e-9,"FFT and probe must agree at a tile node");
    (dummy,eastPlus,ds):=Marine.Environment.SpectralSea.field(8,11,.4,45000,13.7,0,{8.001,4,0},.45);
    (dummy,eastMinus,ds):=Marine.Environment.SpectralSea.field(8,11,.4,45000,13.7,0,{7.999,4,0},.45);
    assert(abs((eastPlus[1]-eastMinus[1])/.002-slope[1])<1e-6,"East slope must differentiate the same wave field");
    currentENU:=Marine.Environment.Current.meanVelocity(0,0,10,0,.2,{.3,.1},.5);
    assert(max(abs(currentENU-{.32,.05,0}))<1e-10,"Current combines residual, tide, and wind in ENU");
    plane:=Marine.Sailing.jibMastPlane(station,identity(3),{2,0,0});
    assert(abs(plane[4]+2-.065)<1e-10 and abs(plane[1]-1)<1e-10,"At centreline the jib passes ahead of the spar");
    contact:=MultiBodyLie.Collision.boxBoxSupport({1.95,0,0},{1,1,1},identity(3),zeros(3),{1,1,1},identity(3));
    assert(abs(contact[1]+.05)<1e-10 and contact[2]>.999,"Overlapping boxes return signed depth and outward normal");
    passed:=true;
  end check;
initial algorithm
    assert(check(),"Marine physics invariants");
    assert(checkDvl(),"DVL keyframe geometry and correspondence gates");
    assert(checkStayPanelContact(),"Finite forestay contact and balanced reaction");
    assert(checkEchoPacking(),"Imaging sonar packing and bearing isolation");
end MarinePhysicsTests;
