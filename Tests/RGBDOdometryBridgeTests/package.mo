within Tests;
package RGBDOdometryBridgeTests "Odometry bridge: edge values, eligibility, solve exclusion, capture fallback and restart"
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDGraphMeasurements = SLAM.PoseGraph.RGBDGraphMeasurements;
  import RGBDLoopVerification = SLAM.LoopClosure.RGBDLoopVerification;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;

  function rotation "Body-to-world rotation Rz(yaw)*Ry(pitch)*Rx(roll)"
    input Real yaw; input Real pitch; input Real roll;
    output Real R[3,3];
  algorithm
    R := {{cos(yaw),-sin(yaw),0},{sin(yaw),cos(yaw),0},{0,0,1}}
      *{{cos(pitch),0,sin(pitch)},{0,1,0},{-sin(pitch),0,cos(pitch)}}
      *{{1,0,0},{0,cos(roll),-sin(roll)},{0,sin(roll),cos(roll)}};
  end rotation;

  function covariance6 "SPD pose covariance [position; right-local angle] with cross terms"
    input Real scale;
    output Real C[6,6];
  algorithm
    C := scale*{{0.04,0.005,0,0.001,0,0},{0.005,0.03,0.002,0,0.001,0},{0,0.002,0.05,0,0,0.002},
      {0.001,0,0,0.002,0.0002,0},{0,0.001,0,0.0002,0.003,0},{0,0,0.002,0,0,0.0025}};
  end covariance6;

  function frameAt "Valid keyframe at a chosen pose, time, filter epoch and covariance"
    input Integer id;
    input Real position[3];
    input Real R[3,3];
    input Real imageTime;
    input Integer sourceRevision = 1;
    input Integer correctionRevision = 0;
    input Real C[6,6] = covariance6(1.0);
    input Integer generation = 1;
    output RGBDKeyframes.Frame frame;
  algorithm
    frame := Tests.RGBDRetentionTests.syntheticFrame(id,position,generation);
    frame.imageTime := imageTime; frame.bodyRotation := R; frame.poseCovariance := C;
    frame.filterSourceRevision := sourceRevision; frame.filterCorrectionRevision := correctionRevision;
  end frameAt;

  function store "Store a frame, asserting acceptance"
    input RGBDKeyframes.Catalog previous;
    input RGBDKeyframes.Frame frame;
    output RGBDKeyframes.Catalog catalog;
  protected
    Boolean accepted; Integer slot; Integer evicted;
  algorithm
    (catalog,accepted,slot,evicted) := RGBDKeyframes.Store(previous,frame,true);
    assert(accepted,"Test keyframe must be stored");
  end store;

  function pairCatalog "Ids 1 and 2 with chosen second pose, time gap and filter epoch"
    input Real p2[3]; input Real R2[3,3]; input Real dt;
    input Integer source2 = 1; input Integer correction2 = 0;
    input Real C2[6,6] = covariance6(1.5);
    output RGBDKeyframes.Catalog catalog;
  algorithm
    catalog := store(RGBDKeyframes.Empty(1),frameAt(1,{1.0,-2.0,0.5},rotation(0.7,0.2,-0.1),10.0));
    catalog := store(catalog,frameAt(2,p2,R2,10.0+dt,source2,correction2,C2));
  end pairCatalog;

  function expectedCovariance "2(Ji Si Ji' + Jj Sj Jj') + floors, written out independently of OdometryEdge"
    input Real Ri[3,3]; input Real Rj[3,3]; input Real pi[3]; input Real pj[3];
    input Real Si[6,6]; input Real Sj[6,6];
    output Real C[6,6];
  protected
    Real u[3]; Real D[3,3]; Real A[6,6]; Real B[6,6]; Real K[3,3];
  algorithm
    u := transpose(Ri)*(pj-pi); D := transpose(Ri)*Rj; K := RGBDUncertaintySkew(u);
    A := zeros(6,6); B := zeros(6,6);
    for r in 1:3 loop
      for c in 1:3 loop
        A[r,c] := -Ri[c,r]; A[r,c+3] := K[r,c]; A[r+3,c+3] := -D[c,r];
        B[r,c] := Ri[c,r]; B[r+3,c+3] := if r == c then 1.0 else 0.0;
      end for;
    end for;
    C := 2.0*(A*Si*transpose(A)+B*Sj*transpose(B));
    for axis in 1:3 loop
      C[axis,axis] := C[axis,axis]+RGBDGraphMeasurements.bridgePositionFloor^2;
      C[axis+3,axis+3] := C[axis+3,axis+3]+RGBDGraphMeasurements.bridgeAngleFloor^2;
    end for;
  end expectedCovariance;

  function sameProblem "Bit-identical solve inputs"
    input RGBDGraphMeasurements.Problem a;
    input RGBDGraphMeasurements.Problem b;
    output Boolean same;
  algorithm
    same := a.accepted == b.accepted and a.nodeCount == b.nodeCount and a.edgeCount == b.edgeCount
      and a.correlationInflation == b.correlationInflation and a.detachedNodes == b.detachedNodes;
    for n in 1:RGBDGraphMeasurements.nodeCapacity loop
      same := same and a.nodeId[n] == b.nodeId[n] and a.catalogSlot[n] == b.catalogSlot[n] and a.nodeMask[n] == b.nodeMask[n];
      for i in 1:3 loop
        same := same and a.positions[n,i] == b.positions[n,i];
        for j in 1:3 loop same := same and a.rotations[n,i,j] == b.rotations[n,i,j]; end for;
      end for;
    end for;
    for e in 1:RGBDGraphMeasurements.edgeCapacity loop
      same := same and a.edgeMask[e] == b.edgeMask[e] and a.fromNode[e] == b.fromNode[e] and a.toNode[e] == b.toNode[e];
      for i in 1:3 loop
        same := same and a.measuredTranslation[e,i] == b.measuredTranslation[e,i];
        for j in 1:3 loop same := same and a.measuredRotation[e,i,j] == b.measuredRotation[e,i,j]; end for;
      end for;
      for i in 1:6 loop
        for j in 1:6 loop same := same and a.information[e,i,j] == b.information[e,i,j]; end for;
      end for;
    end for;
  end sameProblem;

  function chainGraph "Graph over a catalog: one temporal edge per (parent,child) pair of the given kind"
    input RGBDKeyframes.Catalog catalog;
    input Integer parents[:]; input Integer children[size(parents,1)]; input Integer kinds[size(parents,1)];
    output RGBDGraphMeasurements.State graph;
  algorithm
    graph := RGBDGraphMeasurements.Empty(catalog.generation);
    graph.lastCaptureId := catalog.nextId-1; graph.revision := 1;
    for k in 1:size(parents,1) loop
      graph.edges[k] := Tests.RGBDRetentionTests.measuredEdge(catalog,k,kinds[k],
        RGBDKeyframes.SlotOf(catalog,parents[k]),RGBDKeyframes.SlotOf(catalog,children[k]));
    end for;
    graph.nextEdgeId := size(parents,1)+1;
  end chainGraph;

  function refused "Sequential proposal refused with a given Verify reason"
    input Integer reason;
    output RGBDLoopVerification.Proposal proposal;
  algorithm
    proposal := RGBDLoopVerification.EmptyProposal(7); proposal.rejectionReason := reason;
  end refused;

  function sameGraph "Graph state unchanged (refusal leaves the previous owner)"
    input RGBDGraphMeasurements.State a;
    input RGBDGraphMeasurements.State b;
    output Boolean same;
  algorithm
    same := a.generation == b.generation and a.revision == b.revision and a.lastCaptureId == b.lastCaptureId
      and a.nextEdgeId == b.nextEdgeId;
    for e in 1:RGBDGraphMeasurements.edgeCapacity loop
      same := same and a.edges[e].enabled == b.edges[e].enabled and a.edges[e].id == b.edges[e].id
        and a.edges[e].kind == b.edges[e].kind and a.edges[e].referenceId == b.edges[e].referenceId
        and a.edges[e].currentId == b.edges[e].currentId;
    end for;
    for n in 1:RGBDGraphMeasurements.nodeCapacity loop
      same := same and a.parentless[n] == b.parentless[n] and a.parentlessKind[n] == b.parentlessKind[n];
    end for;
  end sameGraph;

  model BridgeEdgeValues "Edge value, independent covariance and global-frame invariance"
    function run
      input Integer generation "Non-evaluated parameter: keeps the test out of translation-time constant folding";
      output Boolean passed;
    protected
      RGBDKeyframes.Catalog catalog; RGBDKeyframes.Catalog rotated; RGBDGraphMeasurements.Edge edge; RGBDGraphMeasurements.Edge edgeRotated;
      Integer reason; Integer i; Integer j; Real C[6,6]; Real G[3,3]; Real GG[6,6];
      Real Ri[3,3]; Real Rj[3,3];
    algorithm
      catalog := pairCatalog({2.5,-1.0,0.9},rotation(1.2,-0.15,0.25),3.0);
      (edge,reason) := RGBDGraphMeasurements.OdometryEdge(catalog,1,2);
      assert(reason == 0 and edge.enabled and edge.kind == 3 and edge.referenceId == 1 and edge.currentId == 2,"Eligible bridge");
      i := RGBDKeyframes.SlotOf(catalog,1); j := RGBDKeyframes.SlotOf(catalog,2);
      Ri := catalog.bodyRotations[i,:,:]; Rj := catalog.bodyRotations[j,:,:];
      for r in 1:3 loop
        assert(abs(edge.translation[r]-(transpose(Ri)*(catalog.bodyPositions[j,:]-catalog.bodyPositions[i,:]))[r]) <= 1e-12,"u = Ri'(pj-pi)");
        for c in 1:3 loop assert(abs(edge.rotation[r,c]-(transpose(Ri)*Rj)[r,c]) <= 1e-12,"D = Ri'Rj"); end for;
      end for;
      C := expectedCovariance(Ri,Rj,catalog.bodyPositions[i,:],catalog.bodyPositions[j,:],
        catalog.poseCovariances[i,:,:],catalog.poseCovariances[j,:,:]);
      for r in 1:6 loop
        for c in 1:6 loop assert(abs(edge.covariance[r,c]-C[r,c]) <= 1e-12,"Covariance is 2(A+B) plus floors"); end for;
      end for;
      assert(RGBDGraphMeasurements.ValidUncertainty(edge.covariance,edge.information),"Information inverts covariance");
      // Rotate the whole world: R -> G R, p -> G p, position covariance -> G S G'.
      G := rotation(-0.9,0.4,1.1); GG := zeros(6,6); GG[1:3,1:3] := G; GG[4:6,4:6] := identity(3);
      rotated := catalog;
      for slot in {i,j} loop
        rotated.bodyRotations[slot,:,:] := G*catalog.bodyRotations[slot,:,:];
        rotated.bodyPositions[slot,:] := G*catalog.bodyPositions[slot,:];
        rotated.poseCovariances[slot,:,:] := GG*catalog.poseCovariances[slot,:,:]*transpose(GG);
      end for;
      (edgeRotated,reason) := RGBDGraphMeasurements.OdometryEdge(rotated,1,2);
      assert(reason == 0,"Rotated world stays eligible");
      for r in 1:3 loop
        assert(abs(edgeRotated.translation[r]-edge.translation[r]) <= 1e-10,"u is frame-invariant");
        for c in 1:3 loop assert(abs(edgeRotated.rotation[r,c]-edge.rotation[r,c]) <= 1e-10,"D is frame-invariant"); end for;
      end for;
      for r in 1:6 loop
        for c in 1:6 loop assert(abs(edgeRotated.covariance[r,c]-edge.covariance[r,c]) <= 1e-10,"Covariance is frame-invariant"); end for;
      end for;
      passed := true;
    end run;
    // Evaluate=false keeps the catalog workload out of translation-time
    // constant folding (as in RGBDRetentionTests).
    parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
    parameter Boolean passed(fixed=false);
  initial equation
    passed = run(generation);
  end BridgeEdgeValues;

  model BridgeEligibility "Hard bounds at equality and just over, continuity, identity, NaN and non-SPD refusals"
    function reasonOf
      input RGBDKeyframes.Catalog catalog;
      output Integer reason;
    protected
      RGBDGraphMeasurements.Edge edge;
    algorithm
      (edge,reason) := RGBDGraphMeasurements.OdometryEdge(catalog,1,2);
    end reasonOf;
    function run
      input Integer generation "Non-evaluated parameter: keeps the test out of translation-time constant folding";
      output Boolean passed;
    protected
      RGBDKeyframes.Catalog c; Real I3[3,3]; Real p1[3]; Integer slot;
      RGBDGraphMeasurements.Edge edge; Integer reason;
    algorithm
      I3 := identity(3);
      // Translations along the reference x axis; identity rotations isolate one bound at a time.
      c := store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0));
      p1 := {0,0,0};
      assert(reasonOf(store(c,frameAt(2,{1,0,0},I3,10.0+2.4999))) == 4,"Inside the visual retry window: not eligible yet");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},I3,12.5))) == 0,"Gap at the minimum is eligible");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},I3,16.0))) == 0,"Gap at the maximum is eligible");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},I3,16.0001))) == 2,"Gap just over the maximum is a bound refusal");
      assert(reasonOf(store(c,frameAt(2,{4.0,0,0},I3,13.0))) == 0,"Translation at the bound is eligible");
      assert(reasonOf(store(c,frameAt(2,{4.0001,0,0},I3,13.0))) == 2,"Translation just over the bound is refused");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},rotation(0.99,0,0),13.0))) == 0,"Rotation inside the bound is eligible");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},rotation(1.01,0,0),13.0))) == 2,"Rotation over the bound is refused");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},I3,13.0,1,1))) == 1,"A filter commit between the captures breaks continuity");
      assert(reasonOf(store(c,frameAt(2,{1,0,0},I3,13.0,2,0))) == 1,"A source change breaks continuity");
      // A non-SPD frame covariance never reaches the catalog (Store refuses it); a corrupted stored
      // covariance is what can still reach OdometryEdge, and its bridge covariance is refused.
      c := store(c,frameAt(2,{1,0,0},I3,13.0)); slot := RGBDKeyframes.SlotOf(c,2);
      c.poseCovariances[slot,:,:] := -10.0*covariance6(1.0);
      assert(reasonOf(c) == 3,"A non-SPD bridge covariance is refused");
      c := store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0));
      // A corrupted stored position far outside the bound (written after Store) is a bound refusal.
      // Modelica source cannot construct a NaN (0.0/0.0 is a division error in OMC); the source relies on
      // comparisons with NaN being false, so a non-finite value can never satisfy the inclusive bounds.
      c := store(c,frameAt(2,{1,0,0},I3,13.0)); slot := RGBDKeyframes.SlotOf(c,2);
      c.bodyPositions[slot,1] := 1e300;
      assert(reasonOf(c) == 2,"An out-of-range translation never passes a bound");
      // Identity: reversed order, a missing id and a stale generation slot are not eligible.
      c := store(store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0)),frameAt(2,{1,0,0},I3,13.0));
      (edge,reason) := RGBDGraphMeasurements.OdometryEdge(c,2,1); assert(reason == 4,"Reversed ids are refused");
      (edge,reason) := RGBDGraphMeasurements.OdometryEdge(c,1,3); assert(reason == 4,"A missing id is refused");
      slot := RGBDKeyframes.SlotOf(c,1); c.generations[slot] := 0;
      assert(reasonOf(c) == 4,"A stale-generation slot is refused");
      passed := true;
    end run;
    // Evaluate=false keeps the catalog workload out of translation-time
    // constant folding (as in RGBDRetentionTests).
    parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
    parameter Boolean passed(fixed=false);
  initial equation
    passed = run(generation);
  end BridgeEligibility;

  model BridgeTriangleInvariance "A bridge in a mixed triangle never changes the solve inputs"
    function run
      input Integer generation "Non-evaluated parameter: keeps the test out of translation-time constant folding";
      output Boolean passed;
    protected
      RGBDKeyframes.Catalog c; RGBDGraphMeasurements.State g; RGBDGraphMeasurements.State h;
      RGBDGraphMeasurements.Problem a; RGBDGraphMeasurements.Problem b; Real I3[3,3];
    algorithm
      I3 := identity(3);
      c := store(store(store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0)),frameAt(2,{1,0,0},I3,13.0)),
        frameAt(3,{2,0,0},I3,16.0));
      // Bridge 1->2 (kind 3), visual 2->3 (kind 1), loop 1->3 (kind 2).
      g := chainGraph(c,{1,2},{2,3},{3,1});
      g.edges[3] := Tests.RGBDRetentionTests.measuredEdge(c,3,2,RGBDKeyframes.SlotOf(c,1),RGBDKeyframes.SlotOf(c,3));
      g.nextEdgeId := 4;
      assert(RGBDGraphMeasurements.ValidState(c,g),"Mixed triangle is a valid graph");
      a := RGBDGraphMeasurements.PrepareProblem(c,g);
      assert(a.accepted and a.nodeCount == 3 and a.edgeCount == 2 and a.edgeMask[1] == 0.0,"Bridge is not a factor");
      // Change only the bridge's mean and information.
      h := g; h.edges[1].translation := {5.0,-3.0,1.0}; h.edges[1].covariance := 1e-4*identity(6);
      h.edges[1].information := 1e4*identity(6); h.edges[1].rotation := rotation(0.5,0.1,0.0);
      b := RGBDGraphMeasurements.PrepareProblem(c,h);
      assert(sameProblem(a,b),"Solve inputs are bit-identical whatever the bridge says");
      // Bridge-only connection: id 3's component excludes 1 and 2.
      g := chainGraph(c,{1,2},{2,3},{1,3});
      a := RGBDGraphMeasurements.PrepareProblem(c,g);
      assert(a.accepted and a.nodeCount == 1 and a.edgeCount == 0 and a.detachedNodes == 2,"An odometry-only chain never reaches the solve");
      passed := true;
    end run;
    // Evaluate=false keeps the catalog workload out of translation-time
    // constant folding (as in RGBDRetentionTests).
    parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
    parameter Boolean passed(fixed=false);
  initial equation
    passed = run(generation);
  end BridgeTriangleInvariance;

  model BridgeCaptureFallback "Capture: bridge on 7/11 after the retry window, every other refusal stands and leaves state unchanged"
    function capture
      input RGBDKeyframes.Catalog previousCatalog;
      input RGBDGraphMeasurements.State graph;
      input RGBDKeyframes.Frame frame;
      input Integer sequentialReason;
      output RGBDGraphMeasurements.Update result;
    algorithm
      result := RGBDGraphMeasurements.Capture(previousCatalog,store(previousCatalog,frame),graph,refused(sequentialReason),
        Tests.RGBDRetentionTests.pendingLoops({0}),true);
    end capture;
    function run
      input Integer generation "Non-evaluated parameter: keeps the test out of translation-time constant folding";
      output Boolean passed;
    protected
      RGBDKeyframes.Catalog c; RGBDKeyframes.Catalog next; RGBDGraphMeasurements.State g; RGBDGraphMeasurements.Update u;
      Real I3[3,3]; Integer bridges;
    algorithm
      I3 := identity(3);
      c := store(store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0)),frameAt(2,{1,0,0},I3,11.0));
      g := chainGraph(c,{1},{2},{1});
      assert(RGBDGraphMeasurements.ValidState(c,g),"Start graph is valid");
      // Consensus failure (7) after the retry window: bridged.
      u := capture(c,g,frameAt(3,{2,0,0},I3,14.0),7);
      assert(u.accepted and u.admittedOdometry == 1 and u.admittedSequential == 0 and u.bridgeRefusalReason == 0,"Consensus failure is bridged");
      bridges := 0;
      for e in 1:RGBDGraphMeasurements.edgeCapacity loop
        if u.state.edges[e].enabled and u.state.edges[e].kind == 3 then
          bridges := bridges+1;
          assert(u.state.edges[e].referenceId == 2 and u.state.edges[e].currentId == 3,"Bridge joins the latest keyframe to the capture");
        end if;
      end for;
      assert(bridges == 1 and RGBDGraphMeasurements.ValidState(store(c,frameAt(3,{2,0,0},I3,14.0)),u.state),"Bridged graph is valid");
      // The pose screen (11) also falls back.
      u := capture(c,g,frameAt(3,{2,0,0},I3,14.0),11);
      assert(u.accepted and u.admittedOdometry == 1,"Pose-screen refusal is bridged");
      // Inside the retry window: refused, previous state kept.
      u := capture(c,g,frameAt(3,{2,0,0},I3,13.0),7);
      assert(not u.accepted and u.rejectionReason == 4 and sameGraph(u.state,g),"Retry window refuses and keeps state");
      // Every non-consensus Verify refusal stands.
      for reason in {2,3,4,5,6,8,9} loop
        u := capture(c,g,frameAt(3,{2,0,0},I3,14.0),reason);
        assert(not u.accepted and sameGraph(u.state,g),"Only reasons 7 and 11 may fall back");
      end for;
      // Continuity broken and a long gap: a restart root, marked in its slot.
      u := capture(c,g,frameAt(3,{2,0,0},I3,17.5,1,1),7);
      next := store(c,frameAt(3,{2,0,0},I3,17.5,1,1));
      assert(u.accepted and u.admittedRestart == 1 and u.admittedOdometry == 0 and u.bridgeRefusalReason == 1,"Long gap without continuity restarts");
      assert(u.state.parentless[RGBDKeyframes.SlotOf(next,3)] == 3 and u.state.parentlessKind[RGBDKeyframes.SlotOf(next,3)] == 1,"Root marker names id 3");
      assert(RGBDGraphMeasurements.ValidState(next,u.state),"Graph with a restart root is valid");
      // The next capture bridges from the root (same filter epoch as the root).
      g := u.state; c := next;
      u := capture(c,g,frameAt(4,{3,0,0},I3,20.5,1,1),7);
      assert(u.accepted and u.admittedOdometry == 1,"Capture after a restart root bridges to it");
      // Continuity broken but the gap below restartGap: refused, state kept.
      c := store(store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0)),frameAt(2,{1,0,0},I3,11.0));
      g := chainGraph(c,{1},{2},{1});
      u := capture(c,g,frameAt(3,{2,0,0},I3,16.9,1,1),7);
      assert(not u.accepted and sameGraph(u.state,g),"Continuity break inside the restart gap is refused");
      // A wrong sequential reason at a long gap never restarts.
      u := capture(c,g,frameAt(3,{2,0,0},I3,17.5,1,1),5);
      assert(not u.accepted and sameGraph(u.state,g),"Restart needs reason 7 or 11");
      passed := true;
    end run;
    // Evaluate=false keeps the catalog workload out of translation-time
    // constant folding (as in RGBDRetentionTests).
    parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
    parameter Boolean passed(fixed=false);
  initial equation
    passed = run(generation);
  end BridgeCaptureFallback;

  model RestartRootCapacity "The ninth restart root is a bounded refusal (reason 9) with state unchanged"
    function run
      input Integer generation "Non-evaluated parameter: keeps the test out of translation-time constant folding";
      output Boolean passed;
    protected
      RGBDKeyframes.Catalog c; RGBDGraphMeasurements.State g; RGBDGraphMeasurements.Update u; Real I3[3,3]; Integer slot;
    algorithm
      I3 := identity(3);
      c := store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0));
      for id in 2:9 loop c := store(c,frameAt(id,{id,0,0},I3,10.0+7.0*id,1,id)); end for;
      g := RGBDGraphMeasurements.Empty(1); g.lastCaptureId := 9; g.revision := 1;
      for id in 2:9 loop
        slot := RGBDKeyframes.SlotOf(c,id); g.parentless[slot] := id; g.parentlessKind[slot] := 1;
      end for;
      assert(RGBDGraphMeasurements.ValidState(c,g) and RGBDGraphMeasurements.RestartRoots(g) == 8,"Eight roots are valid");
      u := RGBDGraphMeasurements.Capture(c,store(c,frameAt(10,{10,0,0},I3,90.0,1,10)),g,refused(7),
        Tests.RGBDRetentionTests.pendingLoops({0}),true);
      assert(not u.accepted and u.rejectionReason == 9 and sameGraph(u.state,g),"Ninth root is refused, state unchanged");
      passed := true;
    end run;
    // Evaluate=false keeps the catalog workload out of translation-time
    // constant folding (as in RGBDRetentionTests).
    parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
    parameter Boolean passed(fixed=false);
  initial equation
    passed = run(generation);
  end RestartRootCapacity;

  model MarkersAndComposition "Marker invariants, kind range, odometry composition and loop promotion"
    function run
      input Integer generation "Non-evaluated parameter: keeps the test out of translation-time constant folding";
      output Boolean passed;
    protected
      RGBDKeyframes.Catalog c; RGBDKeyframes.Catalog captured; RGBDGraphMeasurements.State g; RGBDGraphMeasurements.State h;
      RGBDGraphMeasurements.State r; Real I3[3,3]; Integer s2; Integer s3; Integer composite;
    algorithm
      I3 := identity(3);
      c := store(store(store(RGBDKeyframes.Empty(1),frameAt(1,{0,0,0},I3,10.0)),frameAt(2,{1,0,0},I3,13.0)),
        frameAt(3,{2,0,0},I3,16.0));
      s2 := RGBDKeyframes.SlotOf(c,2); s3 := RGBDKeyframes.SlotOf(c,3);
      g := chainGraph(c,{1,2},{2,3},{3,1});
      assert(RGBDGraphMeasurements.ValidState(c,g),"Kind 3 is a temporal parent");
      h := g; h.edges[1].kind := 4; assert(not RGBDGraphMeasurements.ValidState(c,h),"Kind 4 is rejected");
      h := g; h.edges[1].enabled := false; assert(not RGBDGraphMeasurements.ValidState(c,h),"A parentless keyframe needs a marker");
      h.parentless[s2] := 2; h.parentlessKind[s2] := 2; assert(RGBDGraphMeasurements.ValidState(c,h),"Loop-anchored marker legitimises it");
      h.parentless[s2] := 3; assert(not RGBDGraphMeasurements.ValidState(c,h),"Marker must name the id in its slot");
      h := g; h.parentless[s3] := 3; h.parentlessKind[s3] := 1; assert(not RGBDGraphMeasurements.ValidState(c,h),"A marked node may not have a parent");
      // Evict id 2 (single child, temporal parent): odometry 1->2 composed with visual 2->3 is odometry.
      captured := c; captured.ids[s2] := 4; captured.occupied[s2] := true;
      c.nextSlot := s2;
      r := RGBDGraphMeasurements.Bridge(c,captured,g);
      composite := 0;
      for e in 1:RGBDGraphMeasurements.edgeCapacity loop
        if r.edges[e].enabled and r.edges[e].referenceId == 1 and r.edges[e].currentId == 3 then composite := e; end if;
      end for;
      assert(composite > 0 and r.edges[composite].kind == 3,"Odometry in either part makes the composite odometry");
      // With a verified loop 1->3 already stored, the loop becomes the visual tree edge instead.
      h := g; h.edges[3] := Tests.RGBDRetentionTests.measuredEdge(c,3,2,RGBDKeyframes.SlotOf(c,1),s3); h.nextEdgeId := 4;
      r := RGBDGraphMeasurements.Bridge(c,captured,h);
      composite := 0;
      for e in 1:RGBDGraphMeasurements.edgeCapacity loop
        if r.edges[e].enabled and r.edges[e].referenceId == 1 and r.edges[e].currentId == 3 then composite := composite+1;
          assert(r.edges[e].kind == 1 and r.edges[e].id == 3,"The stored loop is promoted to the tree edge");
        end if;
      end for;
      assert(composite == 1 and r.nextEdgeId == 4,"No odometry composite is added when a loop covers the pair");
      passed := true;
    end run;
    // Evaluate=false keeps the catalog workload out of translation-time
    // constant folding (as in RGBDRetentionTests).
    parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
    parameter Boolean passed(fixed=false);
  initial equation
    passed = run(generation);
  end MarkersAndComposition;
end RGBDOdometryBridgeTests;
