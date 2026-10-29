within Tests.RGBDRetentionTests;
model RGBDRetentionWrapEvictsShortestGap "Full catalog evicts the shortest merged gap with deterministic ties, never id 1 or the newest 16"
  function run "Store 70 evenly spaced keyframes and check slot, eviction and lookup invariants"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    RGBDKeyframes.Catalog catalog; RGBDKeyframes.Frame frame;
    Boolean accepted; Boolean found; Integer slot; Integer evicted; Integer reason; Integer expectedSlot;
    Integer evictions[6];
  algorithm
    catalog := RGBDKeyframes.Empty(generation); evictions := fill(0,6);
    for id in 1:70 loop
      expectedSlot := catalog.nextSlot;
      // 0.5 m steps are exact binary fractions, so equal gaps tie exactly.
      (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(id,{0.5*(id-1),0.0,0.0},generation),true);
      assert(accepted and reason == 0,"Default retention must store every keyframe");
      assert(slot == expectedSlot,"Store must write the slot it published as nextSlot");
      assert(RGBDKeyframes.ValidHeader(catalog),"Header must stay valid after every Store");
      if id <= RGBDKeyframes.keyframeCapacity then
        assert(slot == id and evicted == 0,"Filling takes the lowest free slot and evicts nothing");
      else
        assert(evicted > 1 and evicted < id-RGBDKeyframes.retainedNewest,"Eviction spares id 1 and the newest ids");
        evictions[id-RGBDKeyframes.keyframeCapacity] := evicted;
        (frame,found,slot) := RGBDKeyframes.Lookup(catalog,catalog.generation,evicted);
        assert(not found and slot == 0,"An evicted id must not be found");
      end if;
      assert(newestRetained(catalog),"Id 1 and the newest ids stay retained");
    end for;
    // Equal gaps keep the older id: 2 goes first, then every other id.
    assert(evictions[1] == 2 and evictions[2] == 4 and evictions[3] == 6
      and evictions[4] == 8 and evictions[5] == 10 and evictions[6] == 12,"Ties must evict the older id");
    assert(occupiedCount(catalog) == RGBDKeyframes.keyframeCapacity,"Full catalog keeps every slot occupied");
    for slot in 1:RGBDKeyframes.keyframeCapacity loop
      (frame,found,expectedSlot) := RGBDKeyframes.Lookup(catalog,catalog.generation,catalog.ids[slot]);
      assert(found and expectedSlot == slot and frame.id == catalog.ids[slot],"Lookup resolves ids through ids[slot]");
    end for;
    assert(RGBDKeyframes.NewestSlot(catalog) >= 1 and catalog.ids[RGBDKeyframes.NewestSlot(catalog)] == 70,"Newest slot holds the newest id");
    assert(RGBDKeyframes.OldestId(catalog) == 1,"Id 1 stays the oldest retained id");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Retention wrap tests did not complete");
end RGBDRetentionWrapEvictsShortestGap;
