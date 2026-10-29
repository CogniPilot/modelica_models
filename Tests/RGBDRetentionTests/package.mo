within Tests;
package RGBDRetentionTests "Place-keeping keyframe retention tests"
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDLoopVerification = SLAM.LoopClosure.RGBDLoopVerification;

  constant Real lapStep = 0.32 "Keyframe spacing along the 80 m office lap (m)";
  constant Real lapPerimeter = 80.0 "Office lap perimeter (m)";

  function syntheticFrame "Valid keyframe with eight unit features at a body position"
    input Integer id;
    input Real position[3];
    input Integer generation = 1;
    output RGBDKeyframes.Frame frame;
  algorithm
    frame := RGBDKeyframes.EmptyFrame();
    frame.generation := generation; frame.id := id; frame.epoch := id; frame.imageTime := 0.3*id; frame.count := 8;
    for feature in 1:8 loop
      frame.enabled[feature] := true;
      frame.pixels[feature,1] := 10*feature; frame.pixels[feature,2] := 10;
      frame.opticalPoint[feature,1] := 0.1*feature; frame.opticalPoint[feature,3] := 2.0;
      frame.descriptor[feature,feature] := sqrt(0.5); frame.descriptor[feature,feature+8] := -sqrt(0.5);
    end for;
    frame.bodyPosition := position;
    frame.histogram[1] := 1.0;
  end syntheticFrame;

  function lapPosition "Point at arc length s on the 25 m by 15 m office rectangle (80 m lap)"
    input Real s;
    output Real position[3];
  protected
    Real u;
  algorithm
    u := mod(s,lapPerimeter);
    if u < 25.0 then position := {u,0.0,0.0};
    elseif u < 40.0 then position := {25.0,u-25.0,0.0};
    elseif u < 65.0 then position := {65.0-u,15.0,0.0};
    else position := {0.0,lapPerimeter-u,0.0};
    end if;
  end lapPosition;

  function measuredEdge "Exact relative-pose edge between two retained catalog slots"
    input RGBDKeyframes.Catalog catalog;
    input Integer id;
    input Integer kind;
    input Integer referenceSlot;
    input Integer currentSlot;
    output RGBDGraphMeasurements.Edge edge;
  protected
    Real Ra[3,3]; Real Rb[3,3];
  algorithm
    Ra := catalog.bodyRotations[referenceSlot,:,:]; Rb := catalog.bodyRotations[currentSlot,:,:];
    edge := RGBDGraphMeasurements.EmptyEdge();
    edge.enabled := true; edge.id := id; edge.kind := kind;
    edge.referenceId := catalog.ids[referenceSlot]; edge.currentId := catalog.ids[currentSlot];
    edge.referenceSlot := referenceSlot; edge.currentSlot := currentSlot;
    edge.referenceEpoch := catalog.epochs[referenceSlot]; edge.currentEpoch := catalog.epochs[currentSlot];
    edge.rotation := transpose(Ra)*Rb;
    edge.translation := transpose(Ra)*(catalog.bodyPositions[currentSlot,:]-catalog.bodyPositions[referenceSlot,:]);
    edge.covariance := 0.01*identity(6); edge.information := 100.0*identity(6);
  end measuredEdge;

  function treeGraph "Consecutive tree over the retained ids plus loop edges, as Capture keeps them"
    input RGBDKeyframes.Catalog catalog;
    input Integer parentId[:] "parentId[id]: consecutive parent id, 0 for none";
    input Integer loopReference[:];
    input Integer loopCurrent[size(loopReference,1)];
    output RGBDGraphMeasurements.State graph;
  protected
    Integer edge; Integer a; Integer b; Integer id;
  algorithm
    graph := RGBDGraphMeasurements.Empty(catalog.generation);
    graph.lastCaptureId := catalog.nextId-1; graph.revision := 1; edge := 0;
    for slot in 1:RGBDKeyframes.keyframeCapacity loop
      if catalog.occupied[slot] then
        id := catalog.ids[slot];
        if id <= size(parentId,1) then
          if parentId[id] >= 1 then
            a := RGBDKeyframes.SlotOf(catalog,parentId[id]);
            if a >= 1 then
              edge := edge+1; graph.edges[edge] := measuredEdge(catalog,edge,1,a,slot);
            end if;
          end if;
        end if;
      end if;
    end for;
    for rank in 1:size(loopReference,1) loop
      a := RGBDKeyframes.SlotOf(catalog,loopReference[rank]);
      b := RGBDKeyframes.SlotOf(catalog,loopCurrent[rank]);
      if a >= 1 and b >= 1 and edge < RGBDGraphMeasurements.edgeCapacity then
        edge := edge+1; graph.edges[edge] := measuredEdge(catalog,edge,2,a,b);
      end if;
    end for;
    graph.nextEdgeId := edge+1;
  end treeGraph;

  function reparent "Bridge bookkeeping: children of an evicted id adopt its parent"
    input Integer previous[:];
    input Integer evictedId;
    output Integer parentId[size(previous,1)];
  algorithm
    parentId := previous;
    if evictedId >= 1 and evictedId <= size(previous,1) then
      for id in 1:size(previous,1) loop
        if previous[id] == evictedId then parentId[id] := previous[evictedId]; end if;
      end for;
    end if;
  end reparent;

  function pendingLoops "Pending loop proposals naming the given reference ids (0 for none)"
    input Integer references[:];
    output RGBDLoopVerification.Proposal loops[RGBDGraphMeasurements.proposalCapacity];
  algorithm
    for rank in 1:RGBDGraphMeasurements.proposalCapacity loop
      loops[rank] := RGBDLoopVerification.EmptyProposal(7);
      if rank <= size(references,1) then
        if references[rank] >= 1 then
          loops[rank].verified := true; loops[rank].referenceId := references[rank];
        end if;
      end if;
    end for;
  end pendingLoops;

  constant Integer loopCapacity = 64 "Loop edges a scripted retention run can accumulate";

  function retentionRun "Store a trajectory with the graph retention mask applied before every Store"
    // Mirrors one graph capture per keyframe: the consecutive tree and loop
    // edges kept so far, the pending consecutive proposal (newest id) and at
    // most one pending loop proposal. Loop references never name the slot due
    // for eviction, as retrieval and loop verification exclude it.
    input Integer count;
    input Boolean onLap "True: office lap at lapStep; false: straight line at lineStep";
    input Real lineStep;
    input Integer scriptedReference[:] "Pending loop reference proposed when the matching scriptedCurrent is stored";
    input Integer scriptedCurrent[size(scriptedReference,1)];
    input Integer loopFrom "First id that proposes final-approach loops";
    input Integer loopEvery "Final-approach loop proposal period in keyframes";
    input Integer earlyLimit "Final-approach loops reference ids up to this one";
    input Real loopRadius "Final-approach loops reference retained ids within this distance (m)";
    input Boolean bridge = RGBDGraphMeasurements.retentionBridging;
    input Integer generation = 1;
    output RGBDKeyframes.Catalog catalog;
    output Integer parentId[count];
    output Integer loopReference[loopCapacity];
    output Integer loopCurrent[loopCapacity];
    output Integer loopCount;
    output Integer refusals "Stores after the catalog filled that did not report reason 0";
    output Integer missingVictims "Masks under which no evictable victim existed";
    output Integer heldEvictions "Evictions of a keyframe the mask held";
    output Integer maxProtected "Largest count of occupied slots that may not be evicted";
    output Integer evictionLog[count] "evictionLog[id]: id evicted by storing id, 0 for none";
  protected
    RGBDGraphMeasurements.State graph;
    RGBDLoopVerification.Proposal sequential;
    Integer tier[RGBDKeyframes.keyframeCapacity]; Integer children[RGBDKeyframes.keyframeCapacity];
    Boolean loopReferenced[RGBDKeyframes.keyframeCapacity];
    Boolean accepted; Boolean evictable; Integer slot; Integer evicted; Integer reason;
    Integer reference; Integer protectedCount; Integer victim; Real best; Real distance; Real position[3];
  algorithm
    catalog := RGBDKeyframes.Empty(generation);
    parentId := fill(0,count); evictionLog := fill(0,count);
    loopReference := fill(0,loopCapacity); loopCurrent := fill(0,loopCapacity);
    loopCount := 0; refusals := 0; missingVictims := 0; heldEvictions := 0; maxProtected := 0;
    for id in 1:count loop
      position := if onLap then lapPosition(lapStep*(id-1)) else {lineStep*(id-1),0.0,0.0};
      graph := treeGraph(catalog,parentId,loopReference,loopCurrent);
      sequential := RGBDLoopVerification.EmptyProposal(7);
      if id > 1 then sequential.verified := true; sequential.referenceId := id-1; end if;
      reference := 0;
      for rank in 1:size(scriptedReference,1) loop
        if scriptedCurrent[rank] == id then reference := scriptedReference[rank]; end if;
      end for;
      if reference == 0 and id >= loopFrom and mod(id-loopFrom,loopEvery) == 0 then
        best := loopRadius;
        for candidate in 1:RGBDKeyframes.keyframeCapacity loop
          if catalog.occupied[candidate] and candidate <> catalog.nextSlot and catalog.ids[candidate] <= earlyLimit then
            distance := sqrt(sum((catalog.bodyPositions[candidate,:]-position).^2));
            if distance < best then best := distance; reference := catalog.ids[candidate]; end if;
          end if;
        end for;
      end if;
      if reference >= 1 and loopCount >= loopCapacity then reference := 0; end if;
      (tier,children,loopReferenced) := RGBDGraphMeasurements.RetentionTiers(catalog,graph,sequential,pendingLoops({reference}),bridge);
      if occupiedCount(catalog) == RGBDKeyframes.keyframeCapacity then
        protectedCount := 0;
        for candidate in 1:RGBDKeyframes.keyframeCapacity loop
          if catalog.ids[candidate] == 1 or catalog.ids[candidate] >= catalog.nextId-RGBDKeyframes.retainedNewest
            or tier[candidate] == RGBDKeyframes.retentionHeld then
            protectedCount := protectedCount+1;
          end if;
        end for;
        maxProtected := max(maxProtected,protectedCount);
        (victim,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
        missingVictims := missingVictims+(if evictable then 0 else 1);
      end if;
      (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(id,position,generation),true,tier);
      if evicted > 0 then
        refusals := refusals+(if accepted and reason == 0 then 0 else 1);
        heldEvictions := heldEvictions+(if tier[slot] == RGBDKeyframes.retentionHeld then 1 else 0);
        evictionLog[id] := evicted;
      elseif not accepted then
        refusals := refusals+1;
      end if;
      if accepted then
        if id > 1 then parentId[id] := id-1; end if;
        parentId := reparent(parentId,evicted);
        if reference >= 1 then
          loopCount := loopCount+1; loopReference[loopCount] := reference; loopCurrent[loopCount] := id;
        end if;
      end if;
    end for;
  end retentionRun;

  function maximumSpacing "Largest distance between retained keyframes adjacent in id order (m)"
    input RGBDKeyframes.Catalog catalog;
    output Real spacing;
  protected
    Integer order[RGBDKeyframes.keyframeCapacity]; Integer count;
  algorithm
    (order,count) := RGBDKeyframes.IdOrder(catalog); spacing := 0.0;
    for rank in 2:RGBDKeyframes.keyframeCapacity loop
      if rank <= count then
        spacing := max(spacing,sqrt(sum((catalog.bodyPositions[order[rank],:]-catalog.bodyPositions[order[rank-1],:]).^2)));
      end if;
    end for;
  end maximumSpacing;

  function nearestRetained "Distance from a point to the nearest retained keyframe (m)"
    input RGBDKeyframes.Catalog catalog;
    input Real position[3];
    output Real distance;
  algorithm
    distance := 1e9;
    for slot in 1:RGBDKeyframes.keyframeCapacity loop
      if catalog.occupied[slot] then
        distance := min(distance,sqrt(sum((catalog.bodyPositions[slot,:]-position).^2)));
      end if;
    end for;
  end nearestRetained;

  function newestRetained "True when id 1 (if stored) and the newest retainedNewest ids are retained"
    input RGBDKeyframes.Catalog catalog;
    output Boolean retained;
  algorithm
    retained := catalog.nextId == 1 or RGBDKeyframes.SlotOf(catalog,1) >= 1;
    for back in 1:RGBDKeyframes.retainedNewest loop
      if catalog.nextId-back >= 1 then
        retained := retained and RGBDKeyframes.SlotOf(catalog,catalog.nextId-back) >= 1;
      end if;
    end for;
  end newestRetained;

  function occupiedCount "Number of occupied catalog slots"
    input RGBDKeyframes.Catalog catalog;
    output Integer count;
  algorithm
    count := 0;
    for slot in 1:RGBDKeyframes.keyframeCapacity loop
      count := count+(if catalog.occupied[slot] then 1 else 0);
    end for;
  end occupiedCount;

  function lineCatalog "Catalog of ids 1..count spaced along x, stored with the default retention"
    input Integer count;
    input Real spacing;
    input Integer generation = 1;
    output RGBDKeyframes.Catalog catalog;
  protected
    Boolean accepted; Integer slot; Integer evicted;
  algorithm
    catalog := RGBDKeyframes.Empty(generation);
    for id in 1:count loop
      (catalog,accepted,slot,evicted) := RGBDKeyframes.Store(catalog,syntheticFrame(id,{spacing*(id-1),0.0,0.0},generation),true);
      assert(accepted,"Synthetic keyframe must be stored");
    end for;
  end lineCatalog;
end RGBDRetentionTests;
