within Tests;
model MarinePhysicsTests "Check Fourier surface sampling, hydrostatic current and contact conventions"
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
end MarinePhysicsTests;
