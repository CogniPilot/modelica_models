within Tests.RGBDRetentionTests;
model RGBDRetentionRefusesWhenOverProtected "An over-protected full catalog refuses Store with the distinct retention reason"
  function run "Hold every candidate, by the literal tree rule or by loops, and check the refusal"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
    RGBDKeyframes.Catalog catalog; RGBDKeyframes.Catalog next; RGBDGraphMeasurements.State graph;
    RGBDLoopVerification.Proposal sequential; Integer parentId[65]; Integer loopReference[24]; Integer loopCurrent[24];
    Integer tier[RGBDKeyframes.keyframeCapacity]; Integer children[RGBDKeyframes.keyframeCapacity];
    Boolean loopReferenced[RGBDKeyframes.keyframeCapacity]; Boolean accepted; Boolean evictable;
    Integer slot; Integer evicted; Integer reason; Integer victim;
  algorithm
    catalog := lineCatalog(RGBDKeyframes.keyframeCapacity,0.5,generation);
    parentId := fill(0,65);
    for id in 2:65 loop parentId[id] := id-1; end for;
    sequential := RGBDLoopVerification.EmptyProposal(7); sequential.verified := true; sequential.referenceId := 64;
    graph := treeGraph(catalog,parentId,{0},{0});
    // Literal rule (no bridging) on a plain chain: id 1 is the only leaf-or-root
    // candidate and the catalog protects it, so nothing is evictable.
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}),false);
    (victim,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
    assert(not evictable and victim >= 1,"No victim exists; the fallback slot keeps the header valid");
    (next,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(65,{32.0,0.0,0.0},generation),true,tier);
    assert(not accepted and reason == RGBDKeyframes.storeRetentionRefused and slot == 0 and evicted == 0,
      "Store refuses with the retention reason");
    assert(next.nextId == catalog.nextId and next.nextSlot == catalog.nextSlot,"A refused Store keeps the header");
    for node in 1:RGBDKeyframes.keyframeCapacity loop
      assert(next.ids[node] == catalog.ids[node] and next.occupied[node] == catalog.occupied[node],
        "A refused Store leaves every slot unchanged");
    end for;
    // Loops holding ids 2..49 (both ends of 2->26 ... 25->49) hold every
    // candidate even with bridging.
    for rank in 1:24 loop loopReference[rank] := rank+1; loopCurrent[rank] := rank+25; end for;
    graph := treeGraph(catalog,parentId,loopReference,loopCurrent);
    assert(RGBDGraphMeasurements.ValidState(catalog,graph),"The loop-heavy graph is valid");
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
    (victim,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
    assert(not evictable,"Loops holding every candidate leave no victim");
    (next,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(65,{32.0,0.0,0.0},generation),true,tier);
    assert(not accepted and reason == RGBDKeyframes.storeRetentionRefused,"Loop over-protection refuses with the retention reason");
    // The refusal is a diagnostic: once retention releases the slot, Store proceeds.
    graph := treeGraph(catalog,parentId,{0},{0});
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
    (next,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(65,{32.0,0.0,0.0},generation),true,tier);
    assert(accepted and reason == 0 and evicted >= 2,"Store proceeds once a victim is evictable");
    // Other rejections keep their own reasons.
    (next,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(66,{32.0,0.0,0.0},generation),true,tier);
    assert(not accepted and reason == 2,"A wrong id is still a candidate rejection");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Refusal checks did not complete");
end RGBDRetentionRefusesWhenOverProtected;
