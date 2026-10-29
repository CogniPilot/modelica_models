within Tests.RGBDRetentionTests;
model RGBDRetentionKeepsStartAndCorners "Office lap: 250 keyframes at 0.32 m keep the start, the corners and 2.5 m spacing in 64 slots"
  function run "Drive the 80 m office lap with the graph mask and check the retained places"
    input Integer generation;
    output Boolean passed;
    output Real spacing "Largest spacing between retained id-order neighbours (m)";
    output Real cornerDistance "Largest distance from the start or a corner to the nearest retained keyframe (m)";
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    RGBDKeyframes.Catalog catalog; Integer parentId[250];
    Integer loopReference[loopCapacity]; Integer loopCurrent[loopCapacity]; Integer loopCount;
    Integer refusals; Integer missingVictims; Integer heldEvictions; Integer maxProtected; Integer evictionLog[250];
  algorithm
    (catalog,parentId,loopReference,loopCurrent,loopCount,refusals,missingVictims,heldEvictions,maxProtected,evictionLog) :=
      retentionRun(250,true,lapStep,fill(0,0),fill(0,0),251,1,0,0.0,true,generation);
    assert(refusals == 0 and missingVictims == 0 and heldEvictions == 0,"Every lap Store must evict an evictable keyframe");
    assert(occupiedCount(catalog) == RGBDKeyframes.keyframeCapacity,"The lap must fill all 64 slots");
    assert(RGBDKeyframes.SlotOf(catalog,1) >= 1,"Id 1 (the start) stays retained");
    assert(newestRetained(catalog),"The newest 16 ids stay retained");
    spacing := maximumSpacing(catalog);
    assert(spacing <= 2.5,"Office lap spacing must stay within 2.5 m");
    cornerDistance := max({nearestRetained(catalog,{0.0,0.0,0.0}),nearestRetained(catalog,{25.0,0.0,0.0}),
      nearestRetained(catalog,{25.0,15.0,0.0}),nearestRetained(catalog,{0.0,15.0,0.0})});
    assert(cornerDistance <= 1.0,"The start and every corner keep a keyframe within 1 m");
    assert(RGBDKeyframes.ValidHeader(catalog),"Header stays valid");
    assert(SLAM.PoseGraph.RGBDGraphMeasurements.ValidState(catalog,treeGraph(catalog,parentId,loopReference,loopCurrent)),
      "Retention leaves a consecutive tree rooted at the oldest keyframe");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
  parameter Real maxSpacing(fixed=false) "Largest retained spacing after the lap (m)";
  parameter Real maxCornerDistance(fixed=false) "Largest start or corner distance to a retained keyframe (m)";
initial equation
  (passed,maxSpacing,maxCornerDistance) = run(generation);
equation
  assert(passed,"Office lap retention checks did not complete");
end RGBDRetentionKeepsStartAndCorners;
