within SLAM.PoseGraph;
// The keyframe catalog owns nodes. This owner retains measured body edges,
// never poses inferred from them. All results are proposals for an outer
// estimator/reference/catalog/map transaction; callers must publish together.
package RGBDGraphMeasurements
  import RGBDCatalogLoopVerification = SLAM.LoopClosure.RGBDCatalogLoopVerification;
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLoopVerification = SLAM.LoopClosure.RGBDLoopVerification;
  import RGBDOpticalPointCovariance = SLAM.Localization.RGBDOpticalPointCovariance;
  import RGBDUncertaintyInverse6 = SLAM.Localization.RGBDUncertaintyInverse6;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;
  import RegistrationPairResidual = Vision.Registration.RegistrationPairResidual;
  import RegistrationWhitenedResidual3 = Vision.Registration.RegistrationWhitenedResidual3;

  constant Integer nodeCapacity = RGBDKeyframes.keyframeCapacity;
  constant Integer edgeCapacity = 256;
  constant Integer proposalCapacity = RGBDKeyframeRetrieval.proposalCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer poseDimension = RGBDKeyframes.poseDimension;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;
  // Loop-anchored capture (isolated comparison). When the consecutive
  // edge fails, agreeing verified loops may admit the capture without it, so a
  // refused capture can no longer pin the catalog to a receding keyframe.
  constant Boolean loopAnchoredCapture = true;
  constant Integer anchorMinimumLoops = 2 "Agreeing verified loops to distinct non-anchored references";
  // Place-keeping retention. A single-child keyframe with a consecutive
  // parent may be evicted after Bridge composes its two consecutive edges;
  // false restricts eviction to leaves and a single-child oldest keyframe.
  constant Boolean retentionBridging = true "Allow eviction of bridged single-child keyframes";
  constant Real anchorConsistencySquared = 9.0;
  constant Real anchorMaximumInnovation = 2.0;
  // Odometry bridge. When the consecutive visual edge fails
  // consensus (7) or the pose screen (11) and no loop anchor admits the
  // capture, the filter's own motion between the two captures admits it with
  // a structural temporal edge (kind 3). Bridges keep the catalog growing and
  // the retention tree whole; they never enter a solve, so only visual edges
  // (kinds 1 and 2) correct poses or the filter. Requires loopAnchoredCapture.
  constant Boolean odometryBridge = true;
  constant Real bridgeMinimumGap = 2.5 "s since the latest retained capture before a bridge: visual retries come first";
  constant Real bridgeMaximumGap = 6.0 "s";
  constant Real bridgeMaximumTranslation = 4.0 "m";
  constant Real bridgeMaximumAngle = 1.0 "rad";
  constant Real bridgePositionFloor = 0.02 "m, added in quadrature to the bridge covariance";
  constant Real bridgeAngleFloor = 0.005 "rad, added in quadrature to the bridge covariance";
  // A capture refused a bridge for continuity, bounds or covariance this long
  // after the latest retained capture starts a new graph component instead.
  constant Real restartGap = 6.0 "s";
  constant Integer rootCapacity = 8 "Retained restart roots; exhaustion is a bounded refusal (Capture reason 9)";

  record Edge
    Boolean enabled;
    Integer id;
    Integer kind "1: consecutive capture; 2: verified loop; 3: odometry bridge (structural, never solved)";
    Integer referenceId; Integer currentId;
    Integer referenceSlot; Integer currentSlot;
    Integer referenceEpoch; Integer currentEpoch;
    Real rotation[dimension,dimension]; Real translation[dimension];
    Real covariance[poseDimension,poseDimension];
    Real information[poseDimension,poseDimension];
  end Edge;

  record State
    Integer generation;
    Integer revision;
    Integer lastCaptureId;
    Integer nextEdgeId;
    Edge edges[edgeCapacity];
    // A retained keyframe newer than the oldest without a temporal parent is
    // legitimate only with a marker in its slot: the id and why it has none.
    Integer parentless[nodeCapacity] "Id admitted without a temporal parent, by catalog slot; 0 none";
    Integer parentlessKind[nodeCapacity] "1 restart root; 2 loop-anchored; 0 none";
  end State;

  function EmptyEdge
    output Edge result;
  algorithm
    result.enabled := false; result.id := 0; result.kind := 0;
    result.referenceId := 0; result.currentId := 0;
    result.referenceSlot := 0; result.currentSlot := 0;
    result.referenceEpoch := -1; result.currentEpoch := -1;
    result.rotation := identity(dimension); result.translation := zeros(dimension);
    result.covariance := zeros(poseDimension,poseDimension);
    result.information := zeros(poseDimension,poseDimension);
  end EmptyEdge;

  function Temporal "Tree edge to the keyframe's parent: visual capture or odometry bridge"
    input Integer kind;
    output Boolean temporal;
  algorithm
    temporal := kind == 1 or kind == 3;
  end Temporal;

  function Visual "Edge carrying image evidence; the only kinds any solve, gauge or correction reads"
    input Integer kind;
    output Boolean visual;
  algorithm
    visual := kind == 1 or kind == 2;
  end Visual;

  function RestartRoots "Retained restart roots"
    input State state;
    output Integer count;
  algorithm
    count := 0;
    for slot in 1:nodeCapacity loop
      count := count+(if state.parentless[slot] > 0 and state.parentlessKind[slot] == 1 then 1 else 0);
    end for;
  end RestartRoots;

  function Empty
    input Integer generation = 1;
    output State result;
  protected
    Edge empty;
  algorithm
    result.generation := generation; result.revision := 0;
    result.lastCaptureId := 0; result.nextEdgeId := 1;
    empty := EmptyEdge();
    for slot in 1:edgeCapacity loop result.edges[slot] := empty; end for;
    result.parentless := fill(0,nodeCapacity); result.parentlessKind := fill(0,nodeCapacity);
  end Empty;

  function Bound
    input RGBDKeyframes.Catalog catalog;
    input Integer id; input Integer slot; input Integer epoch;
    output Boolean valid;
  algorithm
    // Header certification belongs to the caller. Integer domains precede
    // all catalog subscripts. Retention assigns slots, so the slot is checked
    // against the stored id rather than derived from it.
    valid := catalog.nextId >= 1 and catalog.nextId <= identifierLimit
      and catalog.lastEpoch >= -1 and catalog.lastEpoch <= identifierLimit;
    if valid then
      valid := id >= 1 and id < catalog.nextId
        and slot >= 1 and slot <= nodeCapacity and epoch >= 0 and epoch <= catalog.lastEpoch;
    end if;
    if valid then
      valid := catalog.occupied[slot] and catalog.ids[slot] == id
        and catalog.generations[slot] == catalog.generation
        and catalog.epochs[slot] == epoch;
    end if;
  end Bound;

  function ValidUncertainty
    input Real covariance[poseDimension,poseDimension];
    input Real information[poseDimension,poseDimension];
    output Boolean valid;
  protected
    Real inverseCovariance[poseDimension,poseDimension];
    Real inverseInformation[poseDimension,poseDimension];
    Real pivot; Boolean covarianceValid; Boolean informationValid;
  algorithm
    (inverseCovariance,covarianceValid,pivot) := RGBDUncertaintyInverse6(covariance,1e-10);
    (inverseInformation,informationValid,pivot) := RGBDUncertaintyInverse6(information,1e-10);
    valid := covarianceValid and informationValid;
    if valid then
      for row in 1:poseDimension loop
        for column in 1:poseDimension loop
          valid := valid and abs(inverseCovariance[row,column]-information[row,column])
            <= 1e-7*max(1.0,abs(inverseCovariance[row,column]))
            and abs(inverseInformation[row,column]-covariance[row,column])
            <= 1e-7*max(1.0,abs(inverseInformation[row,column]));
        end for;
      end for;
    end if;
  end ValidUncertainty;

  function ValidState
    input RGBDKeyframes.Catalog catalog;
    input State state;
    output Boolean valid;
  protected
    Boolean chain[nodeCapacity]; Boolean edgeValid;
    Integer oldest;
  algorithm
    chain := fill(false,nodeCapacity); oldest := 1;
    valid := RGBDKeyframes.ValidHeader(catalog)
      and state.generation == catalog.generation
      and state.revision >= 0 and state.revision <= identifierLimit
      and state.nextEdgeId >= 1 and state.nextEdgeId <= identifierLimit;
    if valid then
      valid := state.lastCaptureId == catalog.nextId-1;
      oldest := RGBDKeyframes.OldestId(catalog);
      for slot in 1:edgeCapacity loop
        if state.edges[slot].enabled then
          edgeValid := state.edges[slot].id >= 1 and state.edges[slot].id < state.nextEdgeId
            and state.edges[slot].kind >= 1 and state.edges[slot].kind <= 3
            and state.edges[slot].referenceId < state.edges[slot].currentId
            and Bound(catalog,state.edges[slot].referenceId,state.edges[slot].referenceSlot,state.edges[slot].referenceEpoch)
            and Bound(catalog,state.edges[slot].currentId,state.edges[slot].currentSlot,state.edges[slot].currentEpoch)
            and RGBDUncertaintyProper(state.edges[slot].rotation)
            and ValidUncertainty(state.edges[slot].covariance,state.edges[slot].information);
          for axis in 1:dimension loop
            edgeValid := edgeValid and abs(state.edges[slot].translation[axis]) <= 1e6;
          end for;
          if Temporal(state.edges[slot].kind) and edgeValid then
            // Temporal edges form a tree: at most one parent per keyframe,
            // always older (referenceId < currentId above).
            edgeValid := not chain[state.edges[slot].currentSlot];
            chain[state.edges[slot].currentSlot] := true;
          end if;
          valid := valid and edgeValid;
          for earlier in 1:slot-1 loop
            if state.edges[earlier].enabled then
              valid := valid and state.edges[earlier].id <> state.edges[slot].id
                and (state.edges[earlier].referenceId <> state.edges[slot].referenceId
                  or state.edges[earlier].currentId <> state.edges[slot].currentId);
            end if;
          end for;
        end if;
      end for;
      // Every retained keyframe newer than the oldest has exactly one
      // consecutive edge to an older retained keyframe, so retention leaves a
      // tree rooted at the oldest surviving keyframe (RetentionTiers, Bridge).
      // Loop-anchored capture drops this: nodes cut off from the newest
      // keyframe's component stay stored but leave the solve (PrepareProblem).
      // A parentless keyframe needs its marker (restart root or loop-anchored);
      // a marker names a retained keyframe in its own slot with no parent.
      for node in 1:nodeCapacity loop
        if state.parentless[node] > 0 then
          valid := valid and catalog.occupied[node] and catalog.ids[node] == state.parentless[node]
            and (state.parentlessKind[node] == 1 or state.parentlessKind[node] == 2) and not chain[node];
        else valid := valid and state.parentless[node] == 0 and state.parentlessKind[node] == 0;
        end if;
        if catalog.occupied[node] and catalog.ids[node] > oldest then
          valid := valid and (chain[node] or state.parentless[node] == catalog.ids[node]);
        end if;
      end for;
      valid := valid and RestartRoots(state) <= rootCapacity;
    end if;
  end ValidState;

  function Reachable
    "Slots connected to keyframe seed by enabled edges"
    // Slots outside the catalog are skipped; callers that need bound edges
    // check ValidState.
    input RGBDKeyframes.Catalog catalog;
    input State state;
    input Integer seed;
    output Boolean reach[nodeCapacity];
    input Boolean visualOnly = false "Follow only edges a solve may read (kinds 1 and 2)";
  protected
    Boolean changed; Integer a; Integer b;
  algorithm
    reach := fill(false,nodeCapacity);
    for node in 1:nodeCapacity loop
      reach[node] := catalog.occupied[node] and catalog.ids[node] == seed;
    end for;
    // Each sweep that changes anything reaches at least one more node.
    changed := true;
    for pass in 1:nodeCapacity loop
      if changed then
        changed := false;
        for slot in 1:edgeCapacity loop
          a := state.edges[slot].referenceSlot; b := state.edges[slot].currentSlot;
          if state.edges[slot].enabled and (not visualOnly or Visual(state.edges[slot].kind))
            and a >= 1 and a <= nodeCapacity and b >= 1 and b <= nodeCapacity then
            if reach[a] <> reach[b] then reach[a] := true; reach[b] := true; changed := true; end if;
          end if;
        end for;
      end if;
    end for;
  end Reachable;

  function ActiveOldest
    "Gauge id: oldest keyframe in the newest keyframe's component"
    input RGBDKeyframes.Catalog catalog;
    input State state;
    output Integer id;
  protected
    Boolean reach[nodeCapacity];
  algorithm
    id := RGBDKeyframes.OldestId(catalog);
    if loopAnchoredCapture and catalog.nextId > 1 then
      reach := Reachable(catalog,state,catalog.nextId-1,visualOnly=true); id := catalog.nextId-1;
      for node in 1:nodeCapacity loop
        if reach[node] and catalog.ids[node] < id then id := catalog.ids[node]; end if;
      end for;
    end if;
  end ActiveOldest;

  function Anchored
    "Retained keyframe admitted without a consecutive edge"
    input RGBDKeyframes.Catalog catalog;
    input State state;
    input Integer id;
    output Boolean anchored;
  algorithm
    anchored := id > RGBDKeyframes.OldestId(catalog);
    for slot in 1:edgeCapacity loop
      if state.edges[slot].enabled and Temporal(state.edges[slot].kind) and state.edges[slot].currentId == id then
        anchored := false;
      end if;
    end for;
  end Anchored;

  function AnchorImplied
    "Motion screen and implied new keyframe position for one verified loop"
    input RGBDKeyframes.Catalog catalog;
    input Integer currentSlot;
    input RGBDLoopVerification.Proposal proposal;
    input Real minimumPivot;
    output Boolean consistent;
    output Real implied[dimension];
    output Real impliedCovariance[dimension,dimension];
  protected
    Integer slot; Real squared; Real arm[dimension,dimension];
  algorithm
    // Callers pass a bound proposal, so the reference is retained.
    slot := max(1,RGBDKeyframes.SlotOf(catalog,proposal.referenceId));
    (consistent,squared) := RGBDCatalogLoopVerification.PoseConsistent(catalog.bodyPositions[slot,:],
      catalog.bodyPositions[currentSlot,:],catalog.bodyRotations[slot,:,:],
      catalog.poseCovariances[slot,:,:],catalog.poseCovariances[currentSlot,:,:],
      proposal.bodyTranslation,proposal.covariance[1:3,1:3],
      maximumSquared=anchorConsistencySquared,minimumPivot=minimumPivot,
      maximumTranslationInnovation=anchorMaximumInnovation);
    implied := catalog.bodyPositions[slot,:]+catalog.bodyRotations[slot,:,:]*proposal.bodyTranslation;
    arm := catalog.bodyRotations[slot,:,:]*RGBDUncertaintySkew(proposal.bodyTranslation);
    impliedCovariance := catalog.poseCovariances[slot,1:3,1:3]
      +catalog.bodyRotations[slot,:,:]*proposal.covariance[1:3,1:3]*transpose(catalog.bodyRotations[slot,:,:])
      +arm*catalog.poseCovariances[slot,4:6,4:6]*transpose(arm);
  end AnchorImplied;

  function AnchorAdmissible
    // The capture is already stored in catalog and graph holds no edge to it.
    // Each agreeing loop passes admission and the production motion screen
    // (forced on), and the loops imply the same new keyframe position.
    input RGBDKeyframes.Catalog catalog;
    input State graph;
    input RGBDLoopVerification.Proposal loops[proposalCapacity];
    input Real inlierDistance; input Real maximumRms;
    input Boolean calibratedResiduals; input Real maximumNormalizedSquared;
    input Real localizationSigma; input Real depthInflation; input Real minimumPivot;
    output Boolean admitted;
    output Boolean evictionRefused;
    output Boolean agreed[proposalCapacity] "Loops the anchor admits; no other loop is inserted";
  protected
    Boolean usable[proposalCapacity]; Boolean consistent; Boolean distinct;
    Integer currentSlot; Integer agreeing; Integer newest;
    Real implied[proposalCapacity,dimension]; Real impliedCovariance[proposalCapacity,dimension,dimension];
    Real squared; Boolean active[nodeCapacity];
    Real position[dimension]; Real positionCovariance[dimension,dimension];
  algorithm
    admitted := false; evictionRefused := false; agreed := fill(false,proposalCapacity);
    // A reference must be in the previous keyframe's component, so an
    // anchor never makes a detached part the active one.
    active := Reachable(catalog,graph,catalog.nextId-2);
    usable := fill(false,proposalCapacity);
    implied := zeros(proposalCapacity,dimension); impliedCovariance := zeros(proposalCapacity,dimension,dimension);
    currentSlot := max(1,RGBDKeyframes.NewestSlot(catalog));
    for rank in 1:proposalCapacity loop
      if loops[rank].verified then
        usable[rank] := ValidProposal(catalog,loops[rank],inlierDistance,maximumRms,
          calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared,
          localizationSigma=localizationSigma,depthInflation=depthInflation,minimumPivot=minimumPivot)
          and not Anchored(catalog,graph,loops[rank].referenceId)
          and active[max(1,RGBDKeyframes.SlotOf(catalog,loops[rank].referenceId))];
        for earlier in 1:rank-1 loop
          usable[rank] := usable[rank] and not (usable[earlier] and loops[earlier].referenceId == loops[rank].referenceId);
        end for;
        if usable[rank] then
          // The proposal goes in as an argument: one reference compiler cannot slice a
          // field of a record array element, and the exporter cannot copy one to a local.
          (consistent,position,positionCovariance) := AnchorImplied(catalog,currentSlot,loops[rank],minimumPivot);
          usable[rank] := consistent; implied[rank,:] := position; impliedCovariance[rank,:,:] := positionCovariance;
        end if;
      end if;
    end for;
    // Agreement is checked against each usable loop in turn: it and every
    // loop whose implied position matches it within the same gate (four
    // times the marginal sum, as in the motion screen).
    for rank in 1:proposalCapacity loop
      if usable[rank] and not admitted then
        agreeing := 0; newest := 0; agreed := fill(false,proposalCapacity);
        for other in 1:proposalCapacity loop
          if usable[other] then
            if other == rank then consistent := true;
            else
              (consistent,squared) := RegistrationWhitenedResidual3(implied[rank,:]-implied[other,:],
                4.0*(impliedCovariance[rank,:,:]+impliedCovariance[other,:,:]),minimumPivot);
              consistent := consistent and squared <= anchorConsistencySquared;
            end if;
            if consistent then agreeing := agreeing+1; newest := max(newest,loops[other].referenceId); end if;
            agreed[other] := consistent;
          end if;
        end for;
        if agreeing >= anchorMinimumLoops then
          // Retention holds every keyframe a verified loop references, so the
          // newest agreeing reference stays retained. Refuse only if it is the
          // slot the next Store overwrites, rather than orphan the anchor.
          if RGBDKeyframes.SlotOf(catalog,newest) <> catalog.nextSlot then admitted := true;
          else evictionRefused := true;
          end if;
        end if;
      end if;
    end for;
    if admitted then evictionRefused := false; else agreed := fill(false,proposalCapacity); end if;
  end AnchorAdmissible;

  function ValidProposal
    input RGBDKeyframes.Catalog catalog;
    input RGBDLoopVerification.Proposal proposal;
    input Real inlierDistance = 0.08;
    input Real maximumRms = 0.03;
    output Boolean valid;
    input Boolean calibratedResiduals = true;
    input Real maximumNormalizedSquared = 9.0;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
  protected
    Integer referenceSlot; Integer currentSlot; Integer partner; Integer inliers;
    Boolean used[RGBDKeyframes.featureCapacity]; Boolean pairValid;
    Real A[dimension,dimension]; Real D[dimension,dimension]; Real u[dimension];
    Real residual[dimension]; Real cost;
    Real referencePoint[dimension]; Real currentPoint[dimension];
    Real referenceCovariance[dimension,dimension]; Real currentCovariance[dimension,dimension];
    Real squared; Boolean residualValid;
  algorithm
    referenceSlot := 1; currentSlot := 1; partner := 1; inliers := 0; cost := 0.0;
    used := fill(false,RGBDKeyframes.featureCapacity);
    valid := RGBDKeyframes.ValidHeader(catalog) and proposal.verified and proposal.rejectionReason == 0
      and proposal.generation == catalog.generation
      and proposal.referenceId >= 1 and proposal.referenceId < proposal.currentId
      and proposal.currentId < catalog.nextId
      and proposal.matchedCount >= proposal.inlierCount
      and proposal.matchedCount <= RGBDKeyframes.featureCapacity
      and proposal.inlierCount >= 3 and proposal.inlierCount <= RGBDKeyframes.featureCapacity
      and proposal.rms >= 0 and proposal.rms <= (if calibratedResiduals then 1e6 else maximumRms)
      and inlierDistance > 0 and inlierDistance <= 10
      and maximumRms >= 0 and maximumRms <= inlierDistance
      and RGBDUncertaintyProper(proposal.opticalRotation)
      and RGBDUncertaintyProper(proposal.bodyRotation)
      and ValidUncertainty(proposal.covariance,proposal.information);
    if calibratedResiduals then
      valid := valid and maximumNormalizedSquared > 0 and maximumNormalizedSquared <= 1e6
        and localizationSigma > 0 and localizationSigma <= 10
        and depthInflation >= 1 and depthInflation <= 100 and minimumPivot > 0 and minimumPivot < 1;
    end if;
    if valid then
      valid := proposal.currentId == catalog.nextId-1;
      // A missing id maps to slot 1, which Bound then refuses.
      referenceSlot := max(1,RGBDKeyframes.SlotOf(catalog,proposal.referenceId));
      currentSlot := max(1,RGBDKeyframes.SlotOf(catalog,proposal.currentId));
      valid := Bound(catalog,proposal.referenceId,referenceSlot,proposal.referenceEpoch)
        and Bound(catalog,proposal.currentId,currentSlot,proposal.currentEpoch);
      if valid and calibratedResiduals then
        // Admission owns its policy and recomputes every residual from the
        // retained calibrated points. A verifier's Boolean is not sufficient.
        valid := catalog.disparityNoises[referenceSlot] > 0 and catalog.disparityNoises[referenceSlot] <= 10
          and catalog.disparityNoises[referenceSlot] == catalog.disparityNoises[currentSlot]
          and catalog.baselines[referenceSlot] > 0 and catalog.baselines[referenceSlot] <= 1
          and catalog.baselines[referenceSlot] == catalog.baselines[currentSlot]
          and catalog.noiseReferenceFocals[referenceSlot] > 0 and catalog.noiseReferenceFocals[referenceSlot] <= 1e6
          and catalog.noiseReferenceFocals[currentSlot] > 0 and catalog.noiseReferenceFocals[currentSlot] <= 1e6;
        for axis in 1:2 loop
          valid := valid and catalog.rgbCalibrations[referenceSlot,axis] > 0 and catalog.rgbCalibrations[referenceSlot,axis] <= 1e6
            and catalog.rgbCalibrations[currentSlot,axis] > 0 and catalog.rgbCalibrations[currentSlot,axis] <= 1e6;
        end for;
      end if;
      if valid then
        A := catalog.opticalToBodyRotations[referenceSlot,:,:]*transpose(proposal.opticalRotation);
        D := A*transpose(catalog.opticalToBodyRotations[currentSlot,:,:]);
        u := catalog.cameraOriginsBody[referenceSlot,:]-A*(proposal.opticalTranslation
          +transpose(catalog.opticalToBodyRotations[currentSlot,:,:])*catalog.cameraOriginsBody[currentSlot,:]);
        valid := max(abs(D-proposal.bodyRotation)) <= 1e-7
          and max(abs(u-proposal.bodyTranslation)) <= 1e-7;
        for axis in 1:dimension loop
          valid := valid and abs(proposal.opticalTranslation[axis]) <= 1e6
            and abs(proposal.bodyTranslation[axis]) <= 1e6;
        end for;
        for feature in 1:RGBDKeyframes.featureCapacity loop
          if proposal.inliers[feature] then
            inliers := inliers+1;
            partner := proposal.partners[feature];
            pairValid := catalog.featureEnabled[referenceSlot,feature]
              and partner >= 1 and partner <= RGBDKeyframes.featureCapacity;
            if pairValid then
              pairValid := catalog.featureEnabled[currentSlot,partner] and not used[partner];
              used[partner] := true;
              referencePoint := catalog.opticalPoints[referenceSlot,feature,:];
              currentPoint := catalog.opticalPoints[currentSlot,partner,:];
              residual := proposal.opticalRotation*referencePoint+proposal.opticalTranslation-currentPoint;
              if calibratedResiduals then
                pairValid := pairValid and referencePoint[3] > 0 and currentPoint[3] > 0
                  and max(abs(referencePoint)) <= 1e6 and max(abs(currentPoint)) <= 1e6;
                if pairValid then
                  referenceCovariance := RGBDOpticalPointCovariance(referencePoint,
                    catalog.rgbCalibrations[referenceSlot,1],catalog.rgbCalibrations[referenceSlot,2],localizationSigma,
                    catalog.disparityNoises[referenceSlot],catalog.noiseReferenceFocals[referenceSlot],catalog.baselines[referenceSlot],depthInflation);
                  currentCovariance := RGBDOpticalPointCovariance(currentPoint,
                    catalog.rgbCalibrations[currentSlot,1],catalog.rgbCalibrations[currentSlot,2],localizationSigma,
                    catalog.disparityNoises[currentSlot],catalog.noiseReferenceFocals[currentSlot],catalog.baselines[currentSlot],depthInflation);
                  (residualValid,squared) := RegistrationPairResidual(referencePoint,currentPoint,
                    proposal.opticalRotation,proposal.opticalTranslation,true,referenceCovariance,currentCovariance,minimumPivot);
                  pairValid := residualValid and squared <= maximumNormalizedSquared;
                end if;
              else
                pairValid := pairValid and residual*residual <= inlierDistance^2;
              end if;
              cost := cost+residual*residual;
            end if;
            valid := valid and pairValid;
          else
            valid := valid and proposal.partners[feature] == 0;
          end if;
        end for;
        valid := valid and inliers == proposal.inlierCount;
        if inliers > 0 then valid := valid and abs(sqrt(cost/inliers)-proposal.rms) <= 1e-6; end if;
      end if;
    end if;
  end ValidProposal;

  function FromProposal
    input RGBDKeyframes.Catalog catalog "Catalog the proposal is bound to";
    input RGBDLoopVerification.Proposal proposal;
    input Integer id; input Integer kind;
    output Edge result;
  algorithm
    result := EmptyEdge(); result.enabled := true; result.id := id; result.kind := kind;
    result.referenceId := proposal.referenceId; result.currentId := proposal.currentId;
    result.referenceSlot := RGBDKeyframes.SlotOf(catalog,proposal.referenceId);
    result.currentSlot := RGBDKeyframes.SlotOf(catalog,proposal.currentId);
    result.referenceEpoch := proposal.referenceEpoch; result.currentEpoch := proposal.currentEpoch;
    result.rotation := proposal.bodyRotation; result.translation := proposal.bodyTranslation;
    result.covariance := proposal.covariance; result.information := proposal.information;
  end FromProposal;

  record Insertion
    State state;
    Boolean accepted; Boolean duplicate;
    Integer replacedLoops;
  end Insertion;

  function Insert
    // Internal primitive: Capture certifies the graph, proposal and kind
    // before calling. Direct callers must satisfy the same preconditions.
    input State previous;
    input RGBDKeyframes.Catalog catalog "Catalog the proposal is bound to";
    input RGBDLoopVerification.Proposal proposal;
    input Integer kind;
    output Insertion result;
  protected
    Integer freeSlot; Integer oldestLoop; Integer oldestId;
  algorithm
    result.state := previous; result.accepted := false; result.duplicate := false; result.replacedLoops := 0;
    freeSlot := 0; oldestLoop := 0; oldestId := identifierLimit;
    for slot in 1:edgeCapacity loop
      if previous.edges[slot].enabled then
        result.duplicate := result.duplicate or (previous.edges[slot].referenceId == proposal.referenceId
          and previous.edges[slot].currentId == proposal.currentId);
        if previous.edges[slot].kind == 2 and previous.edges[slot].id < oldestId then
          oldestLoop := slot; oldestId := previous.edges[slot].id;
        end if;
      elseif freeSlot == 0 then freeSlot := slot;
      end if;
    end for;
    if result.duplicate then
      result.accepted := true;
    elseif previous.nextEdgeId < identifierLimit then
      if freeSlot == 0 then freeSlot := oldestLoop; result.replacedLoops := if oldestLoop > 0 then 1 else 0; end if;
      if freeSlot > 0 then
        result.state.edges[freeSlot] := FromProposal(catalog,proposal,previous.nextEdgeId,kind);
        result.state.nextEdgeId := previous.nextEdgeId+1;
        result.accepted := true;
      end if;
    end if;
  end Insert;

  function InsertEdge
    "Store a prepared edge (an odometry bridge) under Insert's slot and loop-replacement rules"
    // Internal primitive: Capture certifies the graph and the edge first; the
    // edge's id is assigned here.
    input State previous;
    input Edge edge;
    output Insertion result;
  protected
    Integer freeSlot; Integer oldestLoop; Integer oldestId;
  algorithm
    result.state := previous; result.accepted := false; result.duplicate := false; result.replacedLoops := 0;
    freeSlot := 0; oldestLoop := 0; oldestId := identifierLimit;
    for slot in 1:edgeCapacity loop
      if previous.edges[slot].enabled then
        result.duplicate := result.duplicate or (previous.edges[slot].referenceId == edge.referenceId
          and previous.edges[slot].currentId == edge.currentId);
        if previous.edges[slot].kind == 2 and previous.edges[slot].id < oldestId then
          oldestLoop := slot; oldestId := previous.edges[slot].id;
        end if;
      elseif freeSlot == 0 then freeSlot := slot;
      end if;
    end for;
    if not result.duplicate and previous.nextEdgeId < identifierLimit then
      if freeSlot == 0 then freeSlot := oldestLoop; result.replacedLoops := if oldestLoop > 0 then 1 else 0; end if;
      if freeSlot > 0 then
        result.state.edges[freeSlot] := edge;
        result.state.edges[freeSlot].id := previous.nextEdgeId;
        result.state.nextEdgeId := previous.nextEdgeId+1;
        result.accepted := true;
      end if;
    end if;
  end InsertEdge;

  function OdometryEdge
    "Kind-3 temporal edge from the filter's own motion between two retained captures"
    // D = Ri'Rj, u = Ri'(pj-pi) from the raw capture poses (never graph-corrected).
    // Residual [Ri'(pj-pi)-u; Log(D'Ri'Rj)] under p+dp (world), R*Exp(dth):
    //   Ji = [-Ri', skew(u); 0, -D'], Jj = [Ri', 0; 0, I].
    // Covariance 2(Ji Si Ji' + Jj Sj Jj') + floors bounds every correlation
    // between the two filter epochs (first order). No solver reads it (Visual).
    input RGBDKeyframes.Catalog catalog "Holds both captures";
    input Integer referenceId;
    input Integer currentId;
    output Edge result;
    output Integer reason "0 eligible; 1 filter continuity; 2 hard bound; 3 edge covariance; 4 identity/timing";
  protected
    Integer i; Integer j; Boolean valid; Real dt; Real cosine; Real angle; Real pivot;
    Real Ri[dimension,dimension]; Real Rj[dimension,dimension]; Real D[dimension,dimension]; Real u[dimension];
    Real Ji[poseDimension,poseDimension]; Real Jj[poseDimension,poseDimension];
    Real covariance[poseDimension,poseDimension]; Real information[poseDimension,poseDimension];
  algorithm
    result := EmptyEdge(); reason := 4;
    i := RGBDKeyframes.SlotOf(catalog,referenceId); j := RGBDKeyframes.SlotOf(catalog,currentId);
    if i >= 1 and j >= 1 and referenceId < currentId then
      dt := catalog.imageTimes[j]-catalog.imageTimes[i];
      valid := catalog.generations[i] == catalog.generation and catalog.generations[j] == catalog.generation
        and catalog.epochs[i] < catalog.epochs[j] and dt >= bridgeMinimumGap;
      if valid then
        reason := 1;
        if catalog.filterSourceRevisions[i] == catalog.filterSourceRevisions[j]
          and catalog.filterCorrectionRevisions[i] == catalog.filterCorrectionRevisions[j] then
          reason := 2;
          Ri := catalog.bodyRotations[i,:,:]; Rj := catalog.bodyRotations[j,:,:];
          D := transpose(Ri)*Rj; u := transpose(Ri)*(catalog.bodyPositions[j,:]-catalog.bodyPositions[i,:]);
          cosine := min(1.0,max(-1.0,(D[1,1]+D[2,2]+D[3,3]-1.0)/2.0)); angle := acos(cosine);
          // Comparisons with NaN are false, so a non-finite value stays refused.
          if dt <= bridgeMaximumGap and sqrt(u*u) <= bridgeMaximumTranslation and angle <= bridgeMaximumAngle then
            reason := 3;
            Ji := zeros(poseDimension,poseDimension); Jj := zeros(poseDimension,poseDimension);
            Ji[1:3,1:3] := -transpose(Ri); Ji[1:3,4:6] := RGBDUncertaintySkew(u); Ji[4:6,4:6] := -transpose(D);
            Jj[1:3,1:3] := transpose(Ri); Jj[4:6,4:6] := identity(dimension);
            covariance := 2.0*(Ji*catalog.poseCovariances[i,:,:]*transpose(Ji)+Jj*catalog.poseCovariances[j,:,:]*transpose(Jj));
            for axis in 1:dimension loop
              covariance[axis,axis] := covariance[axis,axis]+bridgePositionFloor^2;
              covariance[axis+3,axis+3] := covariance[axis+3,axis+3]+bridgeAngleFloor^2;
            end for;
            covariance := 0.5*(covariance+transpose(covariance));
            (information,valid,pivot) := RGBDUncertaintyInverse6(covariance,1e-10);
            if valid then
              information := 0.5*(information+transpose(information));
              valid := ValidUncertainty(covariance,information);
            end if;
            if valid then
              reason := 0;
              result.enabled := true; result.kind := 3;
              result.referenceId := referenceId; result.currentId := currentId;
              result.referenceSlot := i; result.currentSlot := j;
              result.referenceEpoch := catalog.epochs[i]; result.currentEpoch := catalog.epochs[j];
              result.rotation := D; result.translation := u;
              result.covariance := covariance; result.information := information;
            end if;
          end if;
        end if;
      end if;
    end if;
  end OdometryEdge;

  function RetentionTiers "Per-slot eviction tier the graph grants the keyframe catalog before Store"
    // The catalog never reads graph records; this mask is the whole interface.
    // Held: a verified loop (enabled edge or pending proposal) or the pending
    // consecutive proposal references the keyframe, or removing it would
    // split the consecutive tree. First: a leaf, or the oldest retained
    // keyframe with exactly one child. Bridged: one child and a consecutive
    // parent; Capture composes the two edges. Newest ids and id 1 are
    // protected by the catalog itself.
    // Store checks the tier of the slot due for eviction (nextSlot) and then
    // chooses the next victim with the same mask, so every other slot gets
    // its tier in the graph as it stands after that eviction: the parent of
    // an evicted leaf loses a child, and the only child of an evicted root
    // becomes the root. Evictions only remove children, so these tiers are
    // never less protective than the next capture's own mask.
    input RGBDKeyframes.Catalog catalog;
    input State state;
    input RGBDLoopVerification.Proposal sequential "Pending consecutive proposal; its reference gains a child";
    input RGBDLoopVerification.Proposal loops[proposalCapacity] "Pending loop proposals; their references are held";
    input Boolean bridge = retentionBridging;
    input Boolean projectAfterEviction = true "False is planning only; Store still checks the selected slot against the original tiers";
    output Integer tier[nodeCapacity];
    output Integer children[nodeCapacity] "Consecutive (kind-1) children per slot in the current graph";
    output Boolean loopReferenced[nodeCapacity];
  protected
    Boolean parent[nodeCapacity]; Boolean parentAfter[nodeCapacity];
    Integer parentSlot[nodeCapacity]; Integer childSlot[nodeCapacity]; Integer childrenAfter[nodeCapacity];
    Integer a; Integer b; Integer oldest; Integer oldestAfter; Integer slot; Integer victim; Integer referenceSlot;
    Integer count; Integer root; Boolean linked;
  algorithm
    children := fill(0,nodeCapacity); loopReferenced := fill(false,nodeCapacity); parent := fill(false,nodeCapacity);
    parentSlot := fill(0,nodeCapacity); childSlot := fill(0,nodeCapacity);
    for edge in 1:edgeCapacity loop
      a := state.edges[edge].referenceSlot; b := state.edges[edge].currentSlot;
      if state.edges[edge].enabled and a >= 1 and a <= nodeCapacity and b >= 1 and b <= nodeCapacity then
        if catalog.occupied[a] and catalog.ids[a] == state.edges[edge].referenceId
          and catalog.occupied[b] and catalog.ids[b] == state.edges[edge].currentId then
          if Temporal(state.edges[edge].kind) then
            children[a] := children[a]+1; childSlot[a] := b; parent[b] := true; parentSlot[b] := a;
          else
            loopReferenced[a] := true; loopReferenced[b] := true;
          end if;
        end if;
      end if;
    end for;
    referenceSlot := 0;
    if sequential.verified then
      referenceSlot := RGBDKeyframes.SlotOf(catalog,sequential.referenceId);
      if referenceSlot >= 1 then children[referenceSlot] := children[referenceSlot]+1; end if;
    end if;
    for rank in 1:proposalCapacity loop
      if loops[rank].verified then
        slot := RGBDKeyframes.SlotOf(catalog,loops[rank].referenceId);
        if slot >= 1 then loopReferenced[slot] := true; end if;
      end if;
    end for;
    oldest := RGBDKeyframes.OldestId(catalog);
    childrenAfter := children; parentAfter := parent; oldestAfter := oldest; victim := 0;
    if projectAfterEviction and catalog.nextSlot >= 1 and catalog.nextSlot <= nodeCapacity then
      if catalog.occupied[catalog.nextSlot] then victim := catalog.nextSlot; end if;
    end if;
    if victim >= 1 then
      if parentSlot[victim] >= 1 and children[victim] == 0 then
        childrenAfter[parentSlot[victim]] := childrenAfter[parentSlot[victim]]-1;
      end if;
      if parentSlot[victim] == 0 and children[victim] == 1 and childSlot[victim] >= 1 then
        parentAfter[childSlot[victim]] := false;
      end if;
      oldestAfter := catalog.nextId;
      for node in 1:nodeCapacity loop
        if node <> victim and catalog.occupied[node] and catalog.ids[node] < oldestAfter then oldestAfter := catalog.ids[node]; end if;
      end for;
    end if;
    for node in 1:nodeCapacity loop
      count := if node == victim then children[node] else childrenAfter[node];
      linked := if node == victim then parent[node] else parentAfter[node];
      root := if node == victim then oldest else oldestAfter;
      tier[node] := RGBDKeyframes.retentionHeld;
      if not catalog.occupied[node] then
        tier[node] := RGBDKeyframes.retentionFirst;
      elseif not loopReferenced[node] and node <> referenceSlot then
        if count == 0 or (count == 1 and catalog.ids[node] == root) then
          tier[node] := RGBDKeyframes.retentionFirst;
        elseif bridge and count == 1 and linked then
          tier[node] := RGBDKeyframes.retentionBridged;
        end if;
      end if;
    end for;
  end RetentionTiers;


  function PlanNodeCapacity "Bounded source proposal; never an instruction to evict a held node"
    input RGBDKeyframes.Catalog catalog;
    input State previous;
    input RGBDLoopVerification.Proposal sequential;
    input RGBDLoopVerification.Proposal loops[proposalCapacity];
    output Integer slot;
    output State proposed;
    output Integer retiredLoopFactors;
    output Boolean accepted;
  protected
    State topology;
    Integer tier[nodeCapacity]; Integer children[nodeCapacity];
    Boolean referenced[nodeCapacity]; Boolean evictable;
    Integer candidate;
  algorithm
    slot := catalog.nextSlot; proposed := previous; retiredLoopFactors := 0; accepted := false;
    if RGBDKeyframes.ValidHeader(catalog) and ValidState(catalog,previous) then
      // First prefer an already eligible node. This can repair an obsolete
      // planned slot without losing any graph information.
      (tier,children,referenced) := RetentionTiers(catalog,previous,sequential,loops,
        projectAfterEviction=false);
      (candidate,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
      if not evictable then
        // A fixed node budget cannot make every historical loop endpoint
        // immortal. Plan on temporal topology, retaining all pending matches.
        // This shadow is never published: only loops incident to the one
        // selected victim are retired from the actual source proposal below.
        topology := previous;
        for edge in 1:edgeCapacity loop
          if topology.edges[edge].enabled and topology.edges[edge].kind == 2 then
            topology.edges[edge] := EmptyEdge();
          end if;
        end for;
        (tier,children,referenced) := RetentionTiers(catalog,topology,sequential,loops,
          projectAfterEviction=false);
        (candidate,evictable) := RGBDKeyframes.ChooseVictim(catalog,tier);
        if evictable then
          for edge in 1:edgeCapacity loop
            if proposed.edges[edge].enabled and proposed.edges[edge].kind == 2
                and (proposed.edges[edge].referenceSlot == candidate
                  or proposed.edges[edge].currentSlot == candidate) then
              proposed.edges[edge] := EmptyEdge();
              retiredLoopFactors := retiredLoopFactors+1;
            end if;
          end for;
        end if;
      end if;
      if evictable then
        // No temporal measurement, covariance, identity, root marker or cap
        // changes here. Bridge and Capture still validate the actual removal.
        (tier,children,referenced) := RetentionTiers(catalog,proposed,sequential,loops,
          projectAfterEviction=false);
        accepted := (tier[candidate] == RGBDKeyframes.retentionFirst
            or tier[candidate] == RGBDKeyframes.retentionBridged)
          and ValidState(catalog,proposed);
        if accepted then slot := candidate; end if;
      end if;
      if not accepted then proposed := previous; retiredLoopFactors := 0; end if;
    end if;
  end PlanNodeCapacity;

  function Bridge "Compose the consecutive edges through an evicted single-child keyframe"
    // First-order composition of parent->evicted and evicted->child into one
    // parent->child edge, ordered [translation; rotation] like PGEdge:
    //   R = R1*R2, t = t1+R1*t2,
    //   J1 = [I, -R1*skew(t2); 0, R2'], J2 = [R1, 0; 0, I],
    //   C = J1*C1*J1'+J2*C2*J2'.
    // The composed edge takes a fresh id and the child edge's storage slot;
    // the parent edge is left for the Bound sweep in Capture to remove. A loop
    // edge on the same pair is disabled (a kind-1 edge must carry the tree).
    // Without a parent edge, with no or several children, or on a singular
    // composition nothing changes, and ValidState then rejects the capture.
    input RGBDKeyframes.Catalog previousCatalog;
    input RGBDKeyframes.Catalog capturedCatalog;
    input State previous;
    output State result;
  protected
    Integer victim; Integer evicted; Integer parentEdge; Integer childEdge; Integer children;
    Real R1[dimension,dimension]; Real R2[dimension,dimension];
    Real t1[dimension]; Real t2[dimension];
    Real J1[poseDimension,poseDimension]; Real J2[poseDimension,poseDimension];
    Real C1[poseDimension,poseDimension]; Real C2[poseDimension,poseDimension];
    Real covariance[poseDimension,poseDimension]; Real information[poseDimension,poseDimension];
    Real pivot; Boolean valid; Edge composed; Integer promoted;
  algorithm
    result := previous; evicted := 0; parentEdge := 0; childEdge := 0; children := 0; promoted := 0;
    victim := previousCatalog.nextSlot;
    if victim >= 1 and victim <= nodeCapacity then
      if previousCatalog.occupied[victim] and not (capturedCatalog.occupied[victim]
        and capturedCatalog.ids[victim] == previousCatalog.ids[victim]) then
        evicted := previousCatalog.ids[victim];
      end if;
    end if;
    if evicted > 0 then
      for slot in 1:edgeCapacity loop
        if previous.edges[slot].enabled and Temporal(previous.edges[slot].kind) then
          if previous.edges[slot].currentId == evicted then parentEdge := slot; end if;
          if previous.edges[slot].referenceId == evicted then childEdge := slot; children := children+1; end if;
        end if;
      end for;
    end if;
    if parentEdge > 0 and children == 1 and previous.nextEdgeId < identifierLimit then
      R1 := previous.edges[parentEdge].rotation; t1 := previous.edges[parentEdge].translation;
      R2 := previous.edges[childEdge].rotation; t2 := previous.edges[childEdge].translation;
      J1 := zeros(poseDimension,poseDimension); J2 := zeros(poseDimension,poseDimension);
      J1[1:3,1:3] := identity(dimension); J1[1:3,4:6] := -R1*RGBDUncertaintySkew(t2);
      J1[4:6,4:6] := transpose(R2);
      J2[1:3,1:3] := R1; J2[4:6,4:6] := identity(dimension);
      C1 := previous.edges[parentEdge].covariance; C2 := previous.edges[childEdge].covariance;
      // The two edges share the evicted keyframe's image or filter epoch, so
      // their correlation is unknown: 2(A+B) bounds Cov(a+b) for any.
      covariance := 2.0*(J1*C1*transpose(J1)+J2*C2*transpose(J2));
      covariance := 0.5*(covariance+transpose(covariance));
      (information,valid,pivot) := RGBDUncertaintyInverse6(covariance,1e-10);
      if valid then
        information := 0.5*(information+transpose(information));
        valid := ValidUncertainty(covariance,information);
      end if;
      if valid then
        // Field by field: the exporter cannot copy a record array element.
        // Provenance: the weakest link decides (odometry if either part is).
        composed := EmptyEdge(); composed.enabled := true;
        composed.kind := if previous.edges[parentEdge].kind == 3 or previous.edges[childEdge].kind == 3 then 3 else 1;
        composed.id := previous.nextEdgeId;
        composed.referenceId := previous.edges[parentEdge].referenceId;
        composed.referenceSlot := previous.edges[parentEdge].referenceSlot;
        composed.referenceEpoch := previous.edges[parentEdge].referenceEpoch;
        composed.currentId := previous.edges[childEdge].currentId;
        composed.currentSlot := previous.edges[childEdge].currentSlot;
        composed.currentEpoch := previous.edges[childEdge].currentEpoch;
        composed.rotation := R1*R2; composed.translation := t1+R1*t2;
        composed.covariance := covariance; composed.information := information;
        promoted := 0;
        for slot in 1:edgeCapacity loop
          if previous.edges[slot].enabled and previous.edges[slot].kind == 2
            and previous.edges[slot].referenceId == composed.referenceId
            and previous.edges[slot].currentId == composed.currentId then
            // A verified loop on the pair outranks an odometry composite: it
            // becomes the visual tree edge. A visual composite keeps D2's rule.
            if composed.kind == 3 then promoted := slot; else result.edges[slot] := EmptyEdge(); end if;
          end if;
        end for;
        if promoted > 0 then
          result.edges[promoted].kind := 1; result.edges[childEdge] := EmptyEdge();
        else
          result.edges[childEdge] := composed;
          result.nextEdgeId := previous.nextEdgeId+1;
        end if;
      end if;
    end if;
  end Bridge;

  record Update
    State state;
    Boolean accepted;
    Integer rejectionReason "1 idle; 2 catalog/config; 3 prior graph; 4 chain; 5 loop; 6 capacity/identity; 7 invariant; 8 anchor reference due for eviction; 9 restart root capacity";
    Integer removedEdges; Integer admittedSequential; Integer admittedLoops; Integer duplicateLoops;
    Integer admittedLoopAnchored;
    Integer admittedOdometry; Integer admittedRestart;
    Integer bridgeRefusalReason "OdometryEdge reason when a bridge was tried; 0 otherwise";
  end Update;

  function Capture
    input RGBDKeyframes.Catalog previousCatalog;
    input RGBDKeyframes.Catalog capturedCatalog "Already admitted capture; immutable old payload is trusted";
    input State previous;
    input RGBDLoopVerification.Proposal sequential;
    input RGBDLoopVerification.Proposal loops[proposalCapacity];
    input Boolean requested;
    input Boolean reset = false;
    input Real inlierDistance = 0.08; input Real maximumRms = 0.03;
    output Update result;
    input Boolean calibratedResiduals = true;
    input Real maximumNormalizedSquared = 9.0;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
    input Real loopInlierDistance = inlierDistance;
    input Real loopMaximumRms = maximumRms;
    input Boolean loopCalibratedResiduals = calibratedResiduals;
    input Real loopMaximumNormalizedSquared = maximumNormalizedSquared;
  protected
    State working; Insertion insertion;
    Boolean valid; Integer reason; Integer removed; Integer sequentialCount; Integer loopCount; Integer duplicates;
    Boolean evictionRefused; Integer anchoredCount; Boolean sequentialValid; Boolean admit[proposalCapacity];
    Edge bridgeEdge; Integer bridgeReason; Integer odometryCount; Integer restartCount; Integer currentSlot;
  algorithm
    result.state := previous; result.accepted := false; result.rejectionReason := 1;
    result.removedEdges := 0; result.admittedSequential := 0; result.admittedLoops := 0; result.duplicateLoops := 0;
    result.admittedLoopAnchored := 0; result.admittedOdometry := 0; result.admittedRestart := 0; result.bridgeRefusalReason := 0;
    bridgeReason := 0; odometryCount := 0; restartCount := 0; currentSlot := 0;
    removed := 0; sequentialCount := 0; loopCount := 0; duplicates := 0; anchoredCount := 0; evictionRefused := false; sequentialValid := false;
    admit := fill(true,proposalCapacity);
    if requested then
      reason := 2;
      valid := RGBDKeyframes.ValidHeader(capturedCatalog) and capturedCatalog.nextId >= 2
        and inlierDistance > 0 and inlierDistance <= 10 and maximumRms >= 0 and maximumRms <= inlierDistance;
      if calibratedResiduals then
        valid := valid and maximumNormalizedSquared > 0 and maximumNormalizedSquared <= 1e6
          and localizationSigma > 0 and localizationSigma <= 10
          and depthInflation >= 1 and depthInflation <= 100 and minimumPivot > 0 and minimumPivot < 1;
      end if;
      if reset then
        // A deliberate new generation can recover a damaged old payload.
        valid := valid and previous.generation >= 1 and previous.generation < identifierLimit
          and capturedCatalog.generation > previous.generation and capturedCatalog.nextId == 2;
        working := Empty(capturedCatalog.generation);
      else
        working := previous;
        valid := valid and RGBDKeyframes.ValidHeader(previousCatalog)
          and previousCatalog.nextId < identifierLimit
          and capturedCatalog.generation == previousCatalog.generation
          and capturedCatalog.vocabularyVersion == previousCatalog.vocabularyVersion
          and capturedCatalog.nextId == previousCatalog.nextId+1
          and capturedCatalog.lastEpoch > previousCatalog.lastEpoch
          and (previousCatalog.nextId == 1 or capturedCatalog.lastTime > previousCatalog.lastTime);
        if valid then
          for node in 1:nodeCapacity loop
            if previousCatalog.occupied[node] and node <> previousCatalog.nextSlot then
              valid := valid and capturedCatalog.occupied[node]
                and capturedCatalog.ids[node] == previousCatalog.ids[node]
                and capturedCatalog.epochs[node] == previousCatalog.epochs[node]
                and capturedCatalog.imageTimes[node] == previousCatalog.imageTimes[node];
            end if;
          end for;
          if valid then reason := 3; valid := ValidState(previousCatalog,previous) and previous.revision < identifierLimit; end if;
        end if;
      end if;
      if valid then
        // Keep the consecutive tree connected across the retention eviction.
        if not reset then working := Bridge(previousCatalog,capturedCatalog,working); end if;
        for slot in 1:edgeCapacity loop
          if working.edges[slot].enabled then
            if not Bound(capturedCatalog,working.edges[slot].referenceId,working.edges[slot].referenceSlot,working.edges[slot].referenceEpoch)
              or not Bound(capturedCatalog,working.edges[slot].currentId,working.edges[slot].currentSlot,working.edges[slot].currentEpoch) then
              working.edges[slot] := EmptyEdge(); removed := removed+1;
            end if;
          end if;
        end for;
        // Markers follow their keyframe: an evicted or reused slot loses it.
        for slot in 1:nodeCapacity loop
          if working.parentless[slot] > 0 and not (capturedCatalog.occupied[slot]
              and capturedCatalog.ids[slot] == working.parentless[slot]) then
            working.parentless[slot] := 0; working.parentlessKind[slot] := 0;
          end if;
        end for;
        currentSlot := RGBDKeyframes.SlotOf(capturedCatalog,capturedCatalog.nextId-1);
        reason := 4;
        if not reset and previousCatalog.nextId > 1 then
          valid := sequential.referenceId == previousCatalog.nextId-1
            and ValidProposal(capturedCatalog,sequential,inlierDistance,maximumRms,
              calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared,
              localizationSigma=localizationSigma,depthInflation=depthInflation,minimumPivot=minimumPivot);
          sequentialValid := valid;
          if valid then
            insertion := Insert(working,capturedCatalog,sequential,1); working := insertion.state;
            valid := insertion.accepted and not insertion.duplicate;
            removed := removed+insertion.replacedLoops; sequentialCount := if valid then 1 else 0;
            if not valid then reason := 6; end if;
          end if;
          // A separate branch: this exporter rejects a function call under elseif.
          if loopAnchoredCapture and not sequentialValid then
            (valid,evictionRefused,admit) := AnchorAdmissible(capturedCatalog,working,loops,loopInlierDistance,loopMaximumRms,
              loopCalibratedResiduals,loopMaximumNormalizedSquared,localizationSigma,depthInflation,minimumPivot);
            anchoredCount := if valid then 1 else 0;
            if evictionRefused then reason := 8; end if;
            if valid and currentSlot >= 1 then
              working.parentless[currentSlot] := capturedCatalog.nextId-1; working.parentlessKind[currentSlot] := 2;
            end if;
          end if;
          // Odometry bridge, then restart. Only consensus (7) or the
          // pose screen (11) may fall back; every other refusal stands.
          if odometryBridge and loopAnchoredCapture and not sequentialValid and not valid and not evictionRefused
            and (sequential.rejectionReason == 7 or sequential.rejectionReason == 11) and currentSlot >= 1 then
            (bridgeEdge,bridgeReason) := OdometryEdge(capturedCatalog,previousCatalog.nextId-1,capturedCatalog.nextId-1);
            if bridgeReason == 0 then
              insertion := InsertEdge(working,bridgeEdge); working := insertion.state;
              valid := insertion.accepted and not insertion.duplicate;
              removed := removed+insertion.replacedLoops; odometryCount := if valid then 1 else 0;
              if not valid then reason := 6; end if;
              admit := fill(true,proposalCapacity);
            elseif bridgeReason >= 1 and bridgeReason <= 3
              and capturedCatalog.lastTime-previousCatalog.lastTime >= restartGap then
              if RestartRoots(working) < rootCapacity then
                working.parentless[currentSlot] := capturedCatalog.nextId-1; working.parentlessKind[currentSlot] := 1;
                valid := true; restartCount := 1; admit := fill(true,proposalCapacity);
              else reason := 9;
              end if;
            end if;
          end if;
        else valid := not sequential.verified;
        end if;
        for rank in 1:proposalCapacity loop
          if valid and loops[rank].verified and admit[rank] then
            reason := 5; valid := ValidProposal(capturedCatalog,loops[rank],loopInlierDistance,loopMaximumRms,
              calibratedResiduals=loopCalibratedResiduals,maximumNormalizedSquared=loopMaximumNormalizedSquared,
              localizationSigma=localizationSigma,depthInflation=depthInflation,minimumPivot=minimumPivot);
            if valid then
              insertion := Insert(working,capturedCatalog,loops[rank],2); working := insertion.state; valid := insertion.accepted;
              removed := removed+insertion.replacedLoops;
              duplicates := duplicates+(if insertion.duplicate then 1 else 0);
              loopCount := loopCount+(if insertion.accepted and not insertion.duplicate then 1 else 0);
              if not valid then reason := 6; end if;
            end if;
          end if;
        end for;
        if valid then
          working.lastCaptureId := capturedCatalog.nextId-1;
          working.revision := if reset then 1 else previous.revision+1;
          reason := 7; valid := ValidState(capturedCatalog,working);
        end if;
        if valid then
          result.state := working; result.accepted := true; result.rejectionReason := 0;
          result.removedEdges := removed; result.admittedSequential := sequentialCount;
          result.admittedLoops := loopCount; result.duplicateLoops := duplicates;
          result.admittedLoopAnchored := anchoredCount;
          result.admittedOdometry := odometryCount; result.admittedRestart := restartCount;
          result.bridgeRefusalReason := bridgeReason;
        else result.rejectionReason := reason;
        end if;
      else result.rejectionReason := reason;
      end if;
    end if;
  end Capture;

  record Problem
    Boolean accepted;
    Integer nodeCount; Integer edgeCount; Integer correlationInflation;
    Integer detachedNodes "Stored keyframes cut off from the newest one, out of the solve";
    Integer nodeId[nodeCapacity]; Integer catalogSlot[nodeCapacity];
    Real nodeMask[nodeCapacity]; Real positions[nodeCapacity,dimension];
    Real rotations[nodeCapacity,dimension,dimension];
    Real edgeMask[edgeCapacity]; Real fromNode[edgeCapacity]; Real toNode[edgeCapacity];
    Real measuredRotation[edgeCapacity,dimension,dimension]; Real measuredTranslation[edgeCapacity,dimension];
    Real information[edgeCapacity,poseDimension,poseDimension];
  end Problem;

  function EmptyProblem
    output Problem result;
  algorithm
    result.accepted := false; result.nodeCount := 0; result.edgeCount := 0; result.correlationInflation := 1;
    result.detachedNodes := 0;
    result.nodeId := fill(0,nodeCapacity); result.catalogSlot := fill(0,nodeCapacity);
    result.nodeMask := zeros(nodeCapacity); result.positions := zeros(nodeCapacity,dimension);
    result.edgeMask := zeros(edgeCapacity); result.fromNode := zeros(edgeCapacity); result.toNode := zeros(edgeCapacity);
    result.measuredTranslation := zeros(edgeCapacity,dimension); result.information := zeros(edgeCapacity,poseDimension,poseDimension);
    for node in 1:nodeCapacity loop result.rotations[node,:,:] := identity(dimension); end for;
    for edge in 1:edgeCapacity loop result.measuredRotation[edge,:,:] := identity(dimension); end for;
  end EmptyProblem;

  function PrepareProblem
    input RGBDKeyframes.Catalog catalog;
    input State state;
    input Boolean requested = true;
    output Problem result;
  protected
    Integer slot; Boolean valid; Integer order[nodeCapacity]; Integer count;
    Boolean reach[nodeCapacity]; Integer row[nodeCapacity]; Boolean active[edgeCapacity];
  algorithm
    result := EmptyProblem(); valid := false; order := fill(0,nodeCapacity); count := 0;
    row := fill(0,nodeCapacity);
    if requested then valid := ValidState(catalog,state) and catalog.nextId > 1; end if;
    if valid then
      // Rows follow ascending id; retained ids need not be contiguous.
      (order,count) := RGBDKeyframes.IdOrder(catalog);
      for node in 1:nodeCapacity loop
        if catalog.occupied[node] then
          valid := valid and RGBDUncertaintyProper(catalog.bodyRotations[node,:,:]);
          for axis in 1:dimension loop valid := valid and abs(catalog.bodyPositions[node,axis]) <= 1e6; end for;
        end if;
      end for;
    end if;
    if valid and loopAnchoredCapture then
      // Rows follow id order within the newest keyframe's component, so
      // row 1, the exact gauge, is its oldest node. Detached nodes and the
      // edges among them stay stored and leave the solve.
      reach := Reachable(catalog,state,catalog.nextId-1,visualOnly=true);
      // Rows fill in id order; filling row node after node at a fixed range
      // keeps every index a translation-time loop variable.
      for node in 1:nodeCapacity loop
        if node <= count then
          slot := order[node];
          if reach[slot] then
            result.nodeCount := result.nodeCount+1; row[slot] := result.nodeCount;
          else
            result.detachedNodes := result.detachedNodes+1;
          end if;
        end if;
      end for;
      for node in 1:nodeCapacity loop
        if node <= result.nodeCount then
          // Row node holds the node-th reachable id.
          slot := 0;
          for other in 1:nodeCapacity loop
            if row[other] == node then slot := other; end if;
          end for;
          result.nodeId[node] := catalog.ids[slot]; result.catalogSlot[node] := slot;
          result.nodeMask[node] := 1.0;
          result.positions[node,:] := catalog.bodyPositions[slot,:];
          result.rotations[node,:,:] := catalog.bodyRotations[slot,:,:];
        end if;
      end for;
      // ValidState bound every enabled edge's slots; a disabled edge's are
      // never read.
      active := fill(false,edgeCapacity);
      for edge in 1:edgeCapacity loop
        if state.edges[edge].enabled and Visual(state.edges[edge].kind) then
          active[edge] := reach[state.edges[edge].referenceSlot];
        end if;
        result.edgeCount := result.edgeCount+(if active[edge] then 1 else 0);
      end for;
      result.correlationInflation := max(1,result.edgeCount);
      for edge in 1:edgeCapacity loop
        if active[edge] then
          result.edgeMask[edge] := 1.0;
          result.fromNode[edge] := row[state.edges[edge].referenceSlot];
          result.toNode[edge] := row[state.edges[edge].currentSlot];
          result.measuredRotation[edge,:,:] := state.edges[edge].rotation;
          result.measuredTranslation[edge,:] := state.edges[edge].translation;
          result.information[edge,:,:] := state.edges[edge].information/result.correlationInflation;
        end if;
      end for;
      result.accepted := true;
    end if;
    if valid and not loopAnchoredCapture then
      result.nodeCount := count;
      // A fixed range keeps the loop bounded when the flag folds this branch away.
      for node in 1:nodeCapacity loop
        if node <= result.nodeCount then
          slot := order[node]; row[slot] := node;
          result.nodeId[node] := catalog.ids[slot]; result.catalogSlot[node] := slot; result.nodeMask[node] := 1.0;
          result.positions[node,:] := catalog.bodyPositions[slot,:]; result.rotations[node,:,:] := catalog.bodyRotations[slot,:,:];
        end if;
      end for;
      for edge in 1:edgeCapacity loop
        result.edgeCount := result.edgeCount+(if state.edges[edge].enabled and Visual(state.edges[edge].kind) then 1 else 0);
      end for;
      // No edge independence is presumed. For E arbitrarily correlated
      // errors with bounded marginal covariance R_e, C <= E*diag(R_e).
      // This changes the information scale, never the stored measurement.
      // It does not establish independence from the inertial/filter prior.
      result.correlationInflation := max(1,result.edgeCount);
      for edge in 1:edgeCapacity loop
        if state.edges[edge].enabled and Visual(state.edges[edge].kind) then
          result.edgeMask[edge] := 1.0;
          result.fromNode[edge] := row[state.edges[edge].referenceSlot];
          result.toNode[edge] := row[state.edges[edge].currentSlot];
          result.measuredRotation[edge,:,:] := state.edges[edge].rotation;
          result.measuredTranslation[edge,:] := state.edges[edge].translation;
          result.information[edge,:,:] := state.edges[edge].information/result.correlationInflation;
        end if;
      end for;
      result.accepted := true;
    end if;
  end PrepareProblem;
end RGBDGraphMeasurements;
