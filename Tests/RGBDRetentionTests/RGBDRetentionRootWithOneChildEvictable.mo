within Tests.RGBDRetentionTests;
model RGBDRetentionRootWithOneChildEvictable "The oldest keyframe is evictable with one consecutive child and held with two"
  function run "Thin a catalog whose id 1 is already gone, as after a ring-era restore"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
    RGBDKeyframes.Catalog catalog; RGBDKeyframes.Catalog branched; RGBDGraphMeasurements.State graph;
    RGBDLoopVerification.Proposal sequential; Integer parentId[66]; Integer branchParent[66];
    Integer tier[RGBDKeyframes.keyframeCapacity]; Integer children[RGBDKeyframes.keyframeCapacity];
    Boolean loopReferenced[RGBDKeyframes.keyframeCapacity]; Boolean accepted; Boolean evictable;
    Integer slot; Integer evicted; Integer reason; Integer victim;
  algorithm
    // Only ChooseVictim protects id 1; a header whose nextSlot holds id 1 (a
    // ring-era catalog) still evicts it, leaving id 2 as the oldest.
    catalog := lineCatalog(RGBDKeyframes.keyframeCapacity,0.5,generation);
    catalog.nextSlot := RGBDKeyframes.SlotOf(catalog,1);
    (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(65,{32.0,0.0,0.0},generation),true);
    assert(accepted and evicted == 1 and RGBDKeyframes.OldestId(catalog) == 2,"The ring-era slot evicts id 1");
    parentId := fill(0,66);
    for id in 3:65 loop parentId[id] := id-1; end for;
    sequential := RGBDLoopVerification.EmptyProposal(7); sequential.verified := true; sequential.referenceId := 65;
    // Due for eviction: a mid-chain keyframe away from the root and the branch.
    catalog.nextSlot := RGBDKeyframes.SlotOf(catalog,40);
    // Root 2 with the single child 3: first tier, with or without bridging.
    graph := treeGraph(catalog,parentId,{0},{0});
    assert(RGBDGraphMeasurements.ValidState(catalog,graph),"Chain rooted at id 2 is valid");
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}),false);
    assert(tier[RGBDKeyframes.SlotOf(catalog,2)] == RGBDKeyframes.retentionFirst
      and tier[RGBDKeyframes.SlotOf(catalog,3)] == RGBDKeyframes.retentionHeld,"Without bridging only the root is evictable");
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
    assert(tier[RGBDKeyframes.SlotOf(catalog,2)] == RGBDKeyframes.retentionFirst
      and tier[RGBDKeyframes.SlotOf(catalog,3)] == RGBDKeyframes.retentionBridged,"The root comes before bridged keyframes");
    (victim,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
    assert(evictable and victim == RGBDKeyframes.SlotOf(catalog,2),"The single-child root is the victim");
    // Two children at the root: held; the dead end 3..9 is pruned from 9.
    branchParent := parentId; branchParent[10] := 2;
    graph := treeGraph(catalog,branchParent,{0},{0});
    assert(RGBDGraphMeasurements.ValidState(catalog,graph),"Branched tree rooted at id 2 is valid");
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
    (victim,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
    assert(tier[RGBDKeyframes.SlotOf(catalog,2)] == RGBDKeyframes.retentionHeld,"A root with two children is held");
    assert(evictable and victim == RGBDKeyframes.SlotOf(catalog,9),"The dead-end leaf is the victim instead");
    // Store with the single-child mask once the catalog has chosen the root.
    // The mask already describes the graph after that eviction: id 3 is the
    // next root and evictable first.
    catalog.nextSlot := RGBDKeyframes.SlotOf(catalog,2);
    graph := treeGraph(catalog,parentId,{0},{0});
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
    assert(tier[RGBDKeyframes.SlotOf(catalog,2)] == RGBDKeyframes.retentionFirst
      and tier[RGBDKeyframes.SlotOf(catalog,3)] == RGBDKeyframes.retentionFirst,"After the root goes, its child is the evictable root");
    (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(66,{32.5,0.0,0.0},generation),true,tier);
    assert(accepted and evicted == 2,"The root is evicted");
    parentId[66] := 65; parentId := reparent(parentId,evicted);
    assert(parentId[3] == 0 and RGBDKeyframes.OldestId(catalog) == 3,"Its child becomes the root");
    assert(catalog.nextSlot == RGBDKeyframes.SlotOf(catalog,3),"The new root is the next victim");
    assert(RGBDGraphMeasurements.ValidState(catalog,treeGraph(catalog,parentId,{0},{0})),"The tree stays valid without a bridge");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Root retention checks did not complete");
end RGBDRetentionRootWithOneChildEvictable;
