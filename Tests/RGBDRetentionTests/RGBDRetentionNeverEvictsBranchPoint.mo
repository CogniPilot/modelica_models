within Tests.RGBDRetentionTests;
model RGBDRetentionNeverEvictsBranchPoint "A branch point survives while its children are retained; the dead-end leaf goes first"
  function run "Thin a tree whose branch 21..39 dead-ends while id 40 continues from id 20"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
    constant Integer last = 84;
    RGBDKeyframes.Catalog catalog; RGBDGraphMeasurements.State graph; RGBDLoopVerification.Proposal sequential;
    Integer parentId[last]; Integer tier[RGBDKeyframes.keyframeCapacity]; Integer children[RGBDKeyframes.keyframeCapacity];
    Boolean loopReferenced[RGBDKeyframes.keyframeCapacity]; Boolean accepted; Integer slot; Integer evicted; Integer reason;
  algorithm
    // The mask applies from the Store that fills the catalog (id 64), which
    // chooses the first victim.
    catalog := lineCatalog(RGBDKeyframes.keyframeCapacity-1,0.5,generation);
    parentId := fill(0,last);
    for id in 2:last loop parentId[id] := id-1; end for;
    parentId[40] := 20;
    for id in RGBDKeyframes.keyframeCapacity:last loop
      graph := treeGraph(catalog,parentId,{0},{0});
      assert(RGBDGraphMeasurements.ValidState(catalog,graph),"The branched tree is a valid graph");
      sequential := RGBDLoopVerification.EmptyProposal(7); sequential.verified := true; sequential.referenceId := id-1;
      (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
      // The mask grants tiers for the graph after this Store's eviction, so
      // id 20 is released only by the Store that evicts its last dead-end
      // child (id 21, storing id 83).
      if id > RGBDKeyframes.keyframeCapacity and id < last then
        assert(children[RGBDKeyframes.SlotOf(catalog,20)] == 2,"The branch point has two children until id 21 goes");
        assert(tier[RGBDKeyframes.SlotOf(catalog,20)] == (if id < last-1 then RGBDKeyframes.retentionHeld else RGBDKeyframes.retentionBridged),
          "The branch point is held while both children are retained");
      end if;
      (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(id,{0.5*(id-1),0.0,0.0},generation),true,tier);
      assert(accepted and reason == 0,"The branched tree always has a victim");
      // The dead end is pruned from its tip: 39, 38, ..., 21. Gap ties would
      // have taken id 2 first; the leaf tier comes before spacing.
      if id > RGBDKeyframes.keyframeCapacity and id < last then
        assert(evicted == 39-(id-RGBDKeyframes.keyframeCapacity-1),"Dead-end leaves are evicted tip first");
      end if;
      assert(RGBDKeyframes.SlotOf(catalog,20) >= 1,"The branch point is never evicted while it has two children");
      parentId := reparent(parentId,evicted);
    end for;
    // After the dead end is gone, id 20 is an ordinary bridged chain keyframe.
    assert(evicted <> 20 and RGBDKeyframes.SlotOf(catalog,21) == 0,"The whole dead end is evicted before the branch point");
    graph := treeGraph(catalog,parentId,{0},{0});
    sequential := RGBDLoopVerification.EmptyProposal(7); sequential.verified := true; sequential.referenceId := last;
    (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({0}));
    assert(children[RGBDKeyframes.SlotOf(catalog,20)] == 1 and tier[RGBDKeyframes.SlotOf(catalog,20)] == RGBDKeyframes.retentionBridged,
      "With one child left the former branch point becomes bridgeable");
    assert(RGBDGraphMeasurements.ValidState(catalog,graph),"Pruning keeps the tree valid");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Branch point retention checks did not complete");
end RGBDRetentionNeverEvictsBranchPoint;
