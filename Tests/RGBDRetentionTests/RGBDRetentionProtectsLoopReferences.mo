within Tests.RGBDRetentionTests;
model RGBDRetentionProtectsLoopReferences "Keyframes on a kept or pending loop are never evicted; without the loops they are"
  function run "Store 110 keyframes on a 0.5 m line with and without two loop closures"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    RGBDKeyframes.Catalog catalog; Integer parentId[110];
    Integer loopReference[loopCapacity]; Integer loopCurrent[loopCapacity]; Integer loopCount;
    Integer refusals; Integer missingVictims; Integer heldEvictions; Integer maxProtected; Integer evictionLog[110];
  algorithm
    // Without loops equal gaps evict every even id below the newest 16.
    (catalog,parentId,loopReference,loopCurrent,loopCount,refusals,missingVictims,heldEvictions,maxProtected,evictionLog) :=
      retentionRun(110,false,0.5,fill(0,0),fill(0,0),111,1,0,0.0,true,generation);
    assert(refusals == 0 and missingVictims == 0,"The line run must store every keyframe");
    assert(RGBDKeyframes.SlotOf(catalog,4) == 0 and RGBDKeyframes.SlotOf(catalog,6) == 0
      and RGBDKeyframes.SlotOf(catalog,30) == 0 and RGBDKeyframes.SlotOf(catalog,66) == 0,
      "Without loops ids 4, 6, 30 and 66 are evicted");
    // Loop 4->30 is proposed while storing 30 and kept as an edge; loop 6->66
    // is pending while storing 66, after the catalog is full.
    (catalog,parentId,loopReference,loopCurrent,loopCount,refusals,missingVictims,heldEvictions,maxProtected,evictionLog) :=
      retentionRun(110,false,0.5,{4,6},{30,66},111,1,0,0.0,true,generation);
    assert(loopCount == 2,"Both loops must be kept");
    assert(refusals == 0 and missingVictims == 0 and heldEvictions == 0,"Loop protection must not block the line run");
    assert(RGBDKeyframes.SlotOf(catalog,4) >= 1 and RGBDKeyframes.SlotOf(catalog,6) >= 1
      and RGBDKeyframes.SlotOf(catalog,30) >= 1 and RGBDKeyframes.SlotOf(catalog,66) >= 1,
      "Loop references and loop currents stay retained");
    assert(occupiedCount(catalog) == RGBDKeyframes.keyframeCapacity and newestRetained(catalog),"Full catalog keeps id 1 and the newest 16");
    assert(SLAM.PoseGraph.RGBDGraphMeasurements.ValidState(catalog,treeGraph(catalog,parentId,loopReference,loopCurrent)),
      "Both loop edges stay bound to retained keyframes");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Loop protection checks did not complete");
end RGBDRetentionProtectsLoopReferences;
