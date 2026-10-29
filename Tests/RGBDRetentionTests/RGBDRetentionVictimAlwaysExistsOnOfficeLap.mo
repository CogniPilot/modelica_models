within Tests.RGBDRetentionTests;
model RGBDRetentionVictimAlwaysExistsOnOfficeLap "Office lap with final-approach loop closures never reaches the retention refusal"
  // 300 keyframes at 0.32 m: one 80 m lap plus a 16 m final approach past the
  // start. From id 241 every fourth keyframe proposes a loop to the nearest
  // retained id from the first 30 s (ids up to 100, image time 0.3 s per id)
  // within 1 m; the reference and the current keyframe are then both held.
  function run "Drive the lap with loop protection and check that a victim exists at every full Store"
    input Integer generation;
    output Boolean passed;
    output Integer maxProtected;
    output Integer loopCount;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    RGBDKeyframes.Catalog catalog; Integer parentId[300];
    Integer loopReference[loopCapacity]; Integer loopCurrent[loopCapacity];
    Integer refusals; Integer missingVictims; Integer heldEvictions; Integer evictionLog[300];
  algorithm
    (catalog,parentId,loopReference,loopCurrent,loopCount,refusals,missingVictims,heldEvictions,maxProtected,evictionLog) :=
      retentionRun(300,true,lapStep,fill(0,0),fill(0,0),241,4,100,1.0,true,generation);
    assert(loopCount >= 10,"The final approach must close loops to the first 30 s");
    assert(missingVictims == 0,"An evictable victim must exist at every Store after the catalog fills");
    assert(refusals == 0,"The retention refusal must never occur on the office lap");
    assert(heldEvictions == 0,"A held keyframe must never be evicted");
    // Arithmetic bound: id 1, the newest 16 and every held keyframe together
    // stay below the capacity, so a change to the protected sets trips here.
    assert(maxProtected < RGBDKeyframes.keyframeCapacity,"Protected keyframes must stay strictly below 64");
    for rank in 1:loopCapacity loop
      if rank <= loopCount then
        assert(RGBDKeyframes.SlotOf(catalog,loopReference[rank]) >= 1 and RGBDKeyframes.SlotOf(catalog,loopCurrent[rank]) >= 1,
          "Loop-referenced keyframes stay retained");
      end if;
    end for;
    assert(RGBDKeyframes.SlotOf(catalog,1) >= 1 and newestRetained(catalog),"Id 1 and the newest 16 stay retained");
    assert(SLAM.PoseGraph.RGBDGraphMeasurements.ValidState(catalog,treeGraph(catalog,parentId,loopReference,loopCurrent)),
      "Retention keeps the consecutive tree and every loop edge valid");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
  parameter Integer maxProtected(fixed=false) "Largest protected slot count after the catalog filled";
  parameter Integer loopCount(fixed=false) "Final-approach loop edges kept";
initial equation
  (passed,maxProtected,loopCount) = run(generation);
equation
  assert(passed,"Office lap victim checks did not complete");
end RGBDRetentionVictimAlwaysExistsOnOfficeLap;
