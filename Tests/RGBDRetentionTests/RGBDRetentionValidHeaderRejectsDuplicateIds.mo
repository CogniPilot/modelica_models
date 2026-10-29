within Tests.RGBDRetentionTests;
model RGBDRetentionValidHeaderRejectsDuplicateIds "ValidHeader accepts thinned catalogs and rejects duplicate, misordered or misplaced identities"
  function run "Corrupt a thinned catalog one invariant at a time"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    RGBDKeyframes.Catalog thinned; RGBDKeyframes.Catalog corrupt; RGBDKeyframes.Catalog filling;
    Integer a; Integer b; Integer epoch; Real swapTime;
  algorithm
    thinned := lineCatalog(70,0.5,generation);
    assert(RGBDKeyframes.ValidHeader(thinned),"A thinned catalog with id gaps is valid");
    assert(RGBDKeyframes.SlotOf(thinned,2) == 0,"The thinned catalog has evicted id 2");
    a := RGBDKeyframes.SlotOf(thinned,11); b := RGBDKeyframes.SlotOf(thinned,13);
    assert(a >= 1 and b >= 1,"Ids 11 and 13 are retained");
    // Duplicate id with its own (ordered) epoch and time.
    corrupt := thinned; corrupt.ids[b] := 11;
    assert(not RGBDKeyframes.ValidHeader(corrupt),"Duplicate ids must be rejected");
    // Duplicate id carrying the same epoch and time as the original.
    corrupt := thinned; corrupt.ids[b] := 11; corrupt.epochs[b] := thinned.epochs[a]; corrupt.imageTimes[b] := thinned.imageTimes[a];
    assert(not RGBDKeyframes.ValidHeader(corrupt),"Identical duplicate keyframes must be rejected");
    // Epochs out of id order.
    corrupt := thinned; epoch := corrupt.epochs[a]; corrupt.epochs[a] := corrupt.epochs[b]; corrupt.epochs[b] := epoch;
    assert(not RGBDKeyframes.ValidHeader(corrupt),"Epochs must follow id order");
    corrupt := thinned; swapTime := corrupt.imageTimes[a]; corrupt.imageTimes[a] := corrupt.imageTimes[b]; corrupt.imageTimes[b] := swapTime;
    assert(not RGBDKeyframes.ValidHeader(corrupt),"Image times must follow id order");
    // The slot due for eviction may not hold one of the newest 16 ids.
    corrupt := thinned; corrupt.nextSlot := RGBDKeyframes.NewestSlot(thinned);
    assert(not RGBDKeyframes.ValidHeader(corrupt),"nextSlot must not hold the newest id");
    corrupt := thinned; corrupt.nextSlot := RGBDKeyframes.SlotOf(thinned,thinned.nextId-RGBDKeyframes.retainedNewest);
    assert(not RGBDKeyframes.ValidHeader(corrupt),"nextSlot must not hold any of the newest 16 ids");
    corrupt := thinned; corrupt.nextSlot := RGBDKeyframes.SlotOf(thinned,thinned.nextId-RGBDKeyframes.retainedNewest-1);
    assert(RGBDKeyframes.ValidHeader(corrupt),"nextSlot may hold the newest id outside the protected window");
    // Replacing a newest id by the evicted id 2 (with its own epoch and time)
    // keeps ids unique and ordered but breaks the newest-16 count.
    a := RGBDKeyframes.SlotOf(thinned,thinned.nextId-2);
    corrupt := thinned; corrupt.ids[a] := 2; corrupt.epochs[a] := 2; corrupt.imageTimes[a] := 0.6;
    assert(not RGBDKeyframes.ValidHeader(corrupt),"The newest 16 ids must all be retained");
    // An unoccupied slot in a catalog that should be full.
    corrupt := thinned; corrupt.occupied[a] := false;
    assert(not RGBDKeyframes.ValidHeader(corrupt),"Occupancy must match the stored count");
    // While filling, nextSlot must be free.
    filling := lineCatalog(10,0.5,generation);
    assert(RGBDKeyframes.ValidHeader(filling) and filling.nextSlot == 11,"Filling takes the next free slot");
    corrupt := filling; corrupt.nextSlot := 1;
    assert(not RGBDKeyframes.ValidHeader(corrupt),"nextSlot must be free while filling");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Header identity checks did not complete");
end RGBDRetentionValidHeaderRejectsDuplicateIds;
