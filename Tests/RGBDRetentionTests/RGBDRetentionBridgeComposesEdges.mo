within Tests.RGBDRetentionTests;
model RGBDRetentionBridgeComposesEdges "Evicting a bridged keyframe composes its consecutive edges; the tree admits one parent per keyframe"
  function run "Compose 1->2->3 into 1->3 and check pose, covariance, identity and the single-parent rule"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
    RGBDKeyframes.Catalog catalog; RGBDKeyframes.Catalog previous; RGBDKeyframes.Catalog captured; RGBDKeyframes.Frame frame;
    RGBDGraphMeasurements.State graph; RGBDGraphMeasurements.State result; RGBDGraphMeasurements.State chain;
    Boolean accepted; Integer slot; Integer evicted; Integer reason; Real angle;
    Real R1[3,3]; Real R3[3,3]; Real relative[3,3]; Real expected[3];
  algorithm
    catalog := RGBDKeyframes.Empty(generation);
    for id in 1:3 loop
      angle := 0.4*id;
      frame := syntheticFrame(id,{1.0*id,0.5*id*id,0.1*id},generation);
      frame.bodyRotation := {{cos(angle),-sin(angle),0.0},{sin(angle),cos(angle),0.0},{0.0,0.0,1.0}};
      (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,frame,true);
      assert(accepted and slot == id,"Three keyframes fill slots 1..3");
    end for;
    graph := RGBDGraphMeasurements.Empty(generation);
    graph.edges[1] := measuredEdge(catalog,1,1,1,2);
    graph.edges[2] := measuredEdge(catalog,2,1,2,3);
    graph.edges[3] := measuredEdge(catalog,3,2,1,3);
    graph.nextEdgeId := 4; graph.lastCaptureId := 3; graph.revision := 1;
    assert(RGBDGraphMeasurements.ValidState(catalog,graph),"Chain 1->2->3 with loop 1->3 is valid");
    // Id 2 is due for eviction and the capture overwrote its slot.
    previous := catalog; previous.nextSlot := 2;
    captured := catalog; captured.ids[2] := 4;
    result := RGBDGraphMeasurements.Bridge(previous,captured,graph);
    assert(result.edges[2].enabled and result.edges[2].kind == 1 and result.edges[2].id == 4 and result.nextEdgeId == 5,
      "The composed edge takes the child edge's storage and a fresh id");
    assert(result.edges[2].referenceId == 1 and result.edges[2].referenceSlot == 1 and result.edges[2].referenceEpoch == catalog.epochs[1]
      and result.edges[2].currentId == 3 and result.edges[2].currentSlot == 3 and result.edges[2].currentEpoch == catalog.epochs[3],
      "The composed edge joins the evicted keyframe's parent and child");
    R1 := catalog.bodyRotations[1,:,:]; R3 := catalog.bodyRotations[3,:,:];
    relative := transpose(R1)*R3;
    expected := transpose(R1)*(catalog.bodyPositions[3,:]-catalog.bodyPositions[1,:]);
    for row in 1:3 loop
      assert(abs(result.edges[2].translation[row]-expected[row]) <= 1e-12,"Composed translation is the relative position");
      for column in 1:3 loop
        assert(abs(result.edges[2].rotation[row,column]-relative[row,column]) <= 1e-12,"Composed rotation is the relative rotation");
      end for;
    end for;
    for row in 1:3 loop
      // The edges share the evicted keyframe, so their correlation is unknown and
      // the composite takes the bound 2(A+B): twice the sum of parts.
      assert(result.edges[2].covariance[row,row] >= 0.04-1e-12,"Translation variance bounds both edges and the lever arm");
      for column in 1:3 loop
        assert(abs(result.edges[2].covariance[row+3,column+3]-(if row == column then 0.04 else 0.0)) <= 1e-12,"Rotation variance is twice the sum of both edges");
      end for;
    end for;
    assert(RGBDGraphMeasurements.ValidUncertainty(result.edges[2].covariance,result.edges[2].information),
      "Composed covariance and information are inverse");
    assert(not result.edges[3].enabled,"The loop on the composed pair is dropped");
    assert(result.edges[1].enabled and result.edges[1].currentId == 2,"The parent edge is left for the capture's Bound sweep");
    // A leaf (no child) or a keyframe with two children is not bridged.
    previous.nextSlot := 3; captured := catalog; captured.ids[3] := 4;
    result := RGBDGraphMeasurements.Bridge(previous,captured,graph);
    assert(result.nextEdgeId == 4 and result.edges[2].currentId == 3 and result.edges[3].enabled,"A leaf eviction composes nothing");
    chain := graph; chain.edges[3] := measuredEdge(catalog,3,1,2,3);
    chain.edges[2] := measuredEdge(catalog,2,1,1,3);
    previous.nextSlot := 1; captured := catalog; captured.ids[1] := 4;
    result := RGBDGraphMeasurements.Bridge(previous,captured,chain);
    assert(result.nextEdgeId == 4,"A root without a parent edge composes nothing");
    // Single-parent rule: a second consecutive edge into id 3 is rejected.
    chain := graph; chain.edges[3].kind := 1;
    assert(not RGBDGraphMeasurements.ValidState(catalog,chain),"A keyframe with two consecutive parents is rejected");
    chain := graph; chain.edges[2].enabled := false;
    assert(not RGBDGraphMeasurements.ValidState(catalog,chain),"A keyframe without a consecutive parent is rejected");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Bridge composition checks did not complete");
end RGBDRetentionBridgeComposesEdges;
