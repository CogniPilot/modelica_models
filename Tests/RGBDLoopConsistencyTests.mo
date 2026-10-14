within Tests;
model RGBDLoopConsistencyTests
  function run
    output Boolean passed;
  protected
    import RGBDCatalogLoopVerification = SLAM.LoopClosure.RGBDCatalogLoopVerification;
    Boolean accepted; Real squared;
  algorithm
    // Identical shelf geometry at a different, well-localized bay is an alias.
    (accepted,squared) := RGBDCatalogLoopVerification.PoseConsistent(zeros(3),{6,0,0},identity(3),
      0.25*identity(6),0.25*identity(6),zeros(3),0.01*identity(3));
    assert(not accepted,"Repeated warehouse bay must not become a loop edge");
    (accepted,squared) := RGBDCatalogLoopVerification.PoseConsistent(zeros(3),{0.5,0.1,0},identity(3),
      0.25*identity(6),0.25*identity(6),zeros(3),0.01*identity(3));
    assert(accepted,"A revisited place with ordinary drift must remain eligible");
    (accepted,squared) := RGBDCatalogLoopVerification.PoseConsistent(zeros(3),{6,0,0},identity(3),
      4.0*identity(6),4.0*identity(6),zeros(3),0.01*identity(3),maximumTranslationInnovation=10.0);
    assert(accepted,"Explicit relocalization policy may accept an uncertain large correction");
    passed := true;
  end run;
  parameter Boolean passed = run();
equation
  assert(passed,"Loop consistency tests did not complete");
end RGBDLoopConsistencyTests;
