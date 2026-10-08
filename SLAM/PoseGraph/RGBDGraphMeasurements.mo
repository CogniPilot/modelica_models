within SLAM.PoseGraph;
// The keyframe catalog owns nodes. This owner retains measured body edges,
// never poses inferred from them. All results are proposals for an outer
// estimator/reference/catalog/map transaction; callers must publish together.
package RGBDGraphMeasurements
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLoopVerification = SLAM.LoopClosure.RGBDLoopVerification;
  import RGBDOpticalPointCovariance = SLAM.Localization.RGBDOpticalPointCovariance;
  import RGBDUncertaintyInverse6 = SLAM.Localization.RGBDUncertaintyInverse6;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;
  import RegistrationPairResidual = Vision.Registration.RegistrationPairResidual;

  constant Integer nodeCapacity = RGBDKeyframes.keyframeCapacity;
  constant Integer edgeCapacity = 256;
  constant Integer proposalCapacity = RGBDKeyframeRetrieval.proposalCapacity;
  constant Integer dimension = RGBDKeyframes.dimension;
  constant Integer poseDimension = RGBDKeyframes.poseDimension;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record Edge
    Boolean enabled;
    Integer id;
    Integer kind "1: consecutive capture; 2: verified loop";
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
  end Empty;

  function Bound
    input RGBDKeyframes.Catalog catalog;
    input Integer id; input Integer slot; input Integer epoch;
    output Boolean valid;
  algorithm
    // Header certification belongs to the caller. Integer domains precede
    // subtraction, ring arithmetic and all catalog subscripts.
    valid := catalog.nextId >= 1 and catalog.nextId <= identifierLimit
      and catalog.lastEpoch >= -1 and catalog.lastEpoch <= identifierLimit;
    if valid then
      valid := id >= max(1,catalog.nextId-nodeCapacity) and id < catalog.nextId
        and slot >= 1 and slot <= nodeCapacity and epoch >= 0 and epoch <= catalog.lastEpoch;
    end if;
    if valid then
      valid := mod(id-1,nodeCapacity)+1 == slot;
      if valid then
        valid := catalog.occupied[slot] and catalog.ids[slot] == id
          and catalog.generations[slot] == catalog.generation
          and catalog.epochs[slot] == epoch;
      end if;
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
      oldest := max(1,catalog.nextId-nodeCapacity);
      for slot in 1:edgeCapacity loop
        if state.edges[slot].enabled then
          edgeValid := state.edges[slot].id >= 1 and state.edges[slot].id < state.nextEdgeId
            and (state.edges[slot].kind == 1 or state.edges[slot].kind == 2)
            and state.edges[slot].referenceId < state.edges[slot].currentId
            and Bound(catalog,state.edges[slot].referenceId,state.edges[slot].referenceSlot,state.edges[slot].referenceEpoch)
            and Bound(catalog,state.edges[slot].currentId,state.edges[slot].currentSlot,state.edges[slot].currentEpoch)
            and RGBDUncertaintyProper(state.edges[slot].rotation)
            and ValidUncertainty(state.edges[slot].covariance,state.edges[slot].information);
          for axis in 1:dimension loop
            edgeValid := edgeValid and abs(state.edges[slot].translation[axis]) <= 1e6;
          end for;
          if state.edges[slot].kind == 1 then
            edgeValid := edgeValid and state.edges[slot].currentId == state.edges[slot].referenceId+1;
            if edgeValid then chain[state.edges[slot].currentSlot] := true; end if;
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
      // Every consecutive retained pair has a measured edge, so eviction
      // leaves a connected chain rooted at the oldest surviving keyframe.
      for node in 1:nodeCapacity loop
        if catalog.occupied[node] and catalog.ids[node] > oldest then
          valid := valid and chain[node];
        end if;
      end for;
    end if;
  end ValidState;

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
      referenceSlot := mod(proposal.referenceId-1,nodeCapacity)+1;
      currentSlot := mod(proposal.currentId-1,nodeCapacity)+1;
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
    input RGBDLoopVerification.Proposal proposal;
    input Integer id; input Integer kind;
    output Edge result;
  algorithm
    result := EmptyEdge(); result.enabled := true; result.id := id; result.kind := kind;
    result.referenceId := proposal.referenceId; result.currentId := proposal.currentId;
    result.referenceSlot := mod(proposal.referenceId-1,nodeCapacity)+1;
    result.currentSlot := mod(proposal.currentId-1,nodeCapacity)+1;
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
        result.state.edges[freeSlot] := FromProposal(proposal,previous.nextEdgeId,kind);
        result.state.nextEdgeId := previous.nextEdgeId+1;
        result.accepted := true;
      end if;
    end if;
  end Insert;

  record Update
    State state;
    Boolean accepted;
    Integer rejectionReason "1 idle; 2 catalog/config; 3 prior graph; 4 chain; 5 loop; 6 capacity/identity; 7 invariant";
    Integer removedEdges; Integer admittedSequential; Integer admittedLoops; Integer duplicateLoops;
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
  protected
    State working; Insertion insertion;
    Boolean valid; Integer reason; Integer removed; Integer sequentialCount; Integer loopCount; Integer duplicates;
  algorithm
    result.state := previous; result.accepted := false; result.rejectionReason := 1;
    result.removedEdges := 0; result.admittedSequential := 0; result.admittedLoops := 0; result.duplicateLoops := 0;
    removed := 0; sequentialCount := 0; loopCount := 0; duplicates := 0;
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
        for slot in 1:edgeCapacity loop
          if working.edges[slot].enabled then
            if not Bound(capturedCatalog,working.edges[slot].referenceId,working.edges[slot].referenceSlot,working.edges[slot].referenceEpoch)
              or not Bound(capturedCatalog,working.edges[slot].currentId,working.edges[slot].currentSlot,working.edges[slot].currentEpoch) then
              working.edges[slot] := EmptyEdge(); removed := removed+1;
            end if;
          end if;
        end for;
        reason := 4;
        if not reset and previousCatalog.nextId > 1 then
          valid := sequential.referenceId == previousCatalog.nextId-1
            and ValidProposal(capturedCatalog,sequential,inlierDistance,maximumRms,
              calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared,
              localizationSigma=localizationSigma,depthInflation=depthInflation,minimumPivot=minimumPivot);
          if valid then
            insertion := Insert(working,sequential,1); working := insertion.state;
            valid := insertion.accepted and not insertion.duplicate;
            removed := removed+insertion.replacedLoops; sequentialCount := if valid then 1 else 0;
            if not valid then reason := 6; end if;
          end if;
        else valid := not sequential.verified;
        end if;
        for rank in 1:proposalCapacity loop
          if valid and loops[rank].verified then
            reason := 5; valid := ValidProposal(capturedCatalog,loops[rank],inlierDistance,maximumRms,
              calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared,
              localizationSigma=localizationSigma,depthInflation=depthInflation,minimumPivot=minimumPivot);
            if valid then
              insertion := Insert(working,loops[rank],2); working := insertion.state; valid := insertion.accepted;
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
        else result.rejectionReason := reason;
        end if;
      else result.rejectionReason := reason;
      end if;
    end if;
  end Capture;

  record Problem
    Boolean accepted;
    Integer nodeCount; Integer edgeCount; Integer correlationInflation;
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
    Integer oldest; Integer id; Integer slot; Boolean valid;
  algorithm
    result := EmptyProblem(); valid := false;
    if requested then valid := ValidState(catalog,state) and catalog.nextId > 1; end if;
    if valid then
      for node in 1:nodeCapacity loop
        if catalog.occupied[node] then
          valid := valid and RGBDUncertaintyProper(catalog.bodyRotations[node,:,:]);
          for axis in 1:dimension loop valid := valid and abs(catalog.bodyPositions[node,axis]) <= 1e6; end for;
        end if;
      end for;
    end if;
    if valid then
      result.nodeCount := min(catalog.nextId-1,nodeCapacity); oldest := max(1,catalog.nextId-nodeCapacity);
      for node in 1:result.nodeCount loop
        id := oldest+node-1; slot := mod(id-1,nodeCapacity)+1;
        result.nodeId[node] := id; result.catalogSlot[node] := slot; result.nodeMask[node] := 1.0;
        result.positions[node,:] := catalog.bodyPositions[slot,:]; result.rotations[node,:,:] := catalog.bodyRotations[slot,:,:];
      end for;
      for edge in 1:edgeCapacity loop result.edgeCount := result.edgeCount+(if state.edges[edge].enabled then 1 else 0); end for;
      // No edge independence is presumed. For E arbitrarily correlated
      // errors with bounded marginal covariance R_e, C <= E*diag(R_e).
      // This changes the information scale, never the stored measurement.
      // It does not establish independence from the inertial/filter prior.
      result.correlationInflation := max(1,result.edgeCount);
      for edge in 1:edgeCapacity loop
        if state.edges[edge].enabled then
          result.edgeMask[edge] := 1.0;
          result.fromNode[edge] := state.edges[edge].referenceId-oldest+1;
          result.toNode[edge] := state.edges[edge].currentId-oldest+1;
          result.measuredRotation[edge,:,:] := state.edges[edge].rotation;
          result.measuredTranslation[edge,:] := state.edges[edge].translation;
          result.information[edge,:,:] := state.edges[edge].information/result.correlationInflation;
        end if;
      end for;
      result.accepted := true;
    end if;
  end PrepareProblem;
end RGBDGraphMeasurements;
