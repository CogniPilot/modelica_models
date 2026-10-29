within SLAM.LoopClosure;
// Persistent keyframe ownership, independent of the current single-reference
// filter. A saved histogram and its calibrated geometry share one record/slot.
// These functions are not yet connected to the browser SLAM estimator.
package RGBDKeyframes
  import RGBDUncertaintyInverse6 = SLAM.Localization.RGBDUncertaintyInverse6;
  import RGBDUncertaintyProper = SLAM.Localization.RGBDUncertaintyProper;

  constant Integer imageHeight = 90;
  constant Integer imageWidth = 160;
  constant Integer featureCapacity = 350;
  constant Integer descriptorSize = 49;
  constant Integer wordCapacity = 256;
  constant Integer keyframeCapacity = 64;
  constant Integer dimension = 3;
  constant Integer poseDimension = 6;
  constant Integer identifierLimit = 1000000000;
  // Place-keeping retention. Slots are assigned by Store, never derived from
  // ids; every reader resolves an id through ids[slot] == id.
  constant Integer retainedNewest = 16 "Newest ids never chosen for eviction (local tracking window)";
  constant Integer retentionHeld = 0 "Retention tier: the graph needs this keyframe";
  constant Integer retentionFirst = 1 "Retention tier: evictable without changing any kept edge";
  constant Integer retentionBridged = 2 "Retention tier: evictable after its consecutive edges are composed";
  constant Integer storeRetentionRefused = 3 "Store reason: retention holds the slot due for eviction";

  record Frame
    Integer generation "Reset/source/world ownership generation";
    Integer id "Monotone identity; never a reusable array slot";
    Integer epoch "Measured image identity, independent of wall time";
    Real imageTime "Simulation time of image acquisition";
    Integer count "Full feature domain, including disabled sparse slots";
    Boolean enabled[featureCapacity];
    Real descriptor[featureCapacity,descriptorSize];
    Real opticalPoint[featureCapacity,dimension];
    Integer pixels[featureCapacity,2] "Zero-based RGB pixels";
    Integer rgbSize[2] "Height, width";
    Integer depthSize[2];
    Real rgbCalibration[4] "fx, fy, cx, cy";
    Real depthCalibration[4];
    Real opticalToBody[dimension,dimension];
    Real cameraOriginBody[dimension];
    Real disparityNoise;
    Real noiseReferenceFx;
    Real baseline;
    Real bodyRotation[dimension,dimension];
    Real bodyPosition[dimension];
    Real poseCovariance[poseDimension,poseDimension];
    // Filter epoch of bodyPosition/bodyRotation: two captures share one
    // uncorrected filter frame only when both counters match (odometry bridge).
    Integer filterSourceRevision "Estimator source revision at capture";
    Integer filterCorrectionRevision "Accepted graph-to-filter commits before capture";
    Integer vocabularyVersion;
    Real histogram[wordCapacity];
  end Frame;

  function EmptyFrame
    output Frame frame;
  algorithm
    frame.generation := 1;
    frame.id := 0;
    frame.epoch := -1;
    frame.imageTime := 0.0;
    frame.count := 0;
    frame.enabled := fill(false,featureCapacity);
    frame.descriptor := zeros(featureCapacity,descriptorSize);
    frame.opticalPoint := zeros(featureCapacity,dimension);
    frame.pixels := fill(0,featureCapacity,2);
    frame.rgbSize := {imageHeight,imageWidth};
    frame.depthSize := {imageHeight,imageWidth};
    frame.rgbCalibration := {116.4,116.4,79.5,44.5};
    frame.depthCalibration := {84.3,84.3,79.5,44.5};
    frame.opticalToBody := [0,0,1;-1,0,0;0,-1,0];
    frame.cameraOriginBody := {0.18,0,-0.04};
    frame.disparityNoise := 0.08;
    frame.noiseReferenceFx := 84.3;
    frame.baseline := 0.05;
    frame.bodyRotation := identity(dimension);
    frame.bodyPosition := zeros(dimension);
    frame.poseCovariance := identity(poseDimension);
    frame.filterSourceRevision := 0; frame.filterCorrectionRevision := 0;
    frame.vocabularyVersion := 1;
    frame.histogram := zeros(wordCapacity);
  end EmptyFrame;

  record Catalog
    // Declaration bindings here would add equations to a whole-record model
    // output. Empty owns initialization of every field, including these.
    Integer generation;
    Integer vocabularyVersion "All retained histograms share one vocabulary";
    Integer nextId;
    Integer nextSlot;
    Integer lastEpoch;
    Real lastTime;
    Boolean occupied[keyframeCapacity];
    // Structure-of-arrays storage; Frame remains the public measurement API.
    Integer generations[keyframeCapacity];
    Integer ids[keyframeCapacity];
    Integer epochs[keyframeCapacity];
    Real imageTimes[keyframeCapacity];
    Integer counts[keyframeCapacity];
    Boolean featureEnabled[keyframeCapacity,featureCapacity];
    Real descriptors[keyframeCapacity,featureCapacity,descriptorSize];
    Real opticalPoints[keyframeCapacity,featureCapacity,dimension];
    Integer pixelCoordinates[keyframeCapacity,featureCapacity,2];
    Integer rgbSizes[keyframeCapacity,2];
    Integer depthSizes[keyframeCapacity,2];
    Real rgbCalibrations[keyframeCapacity,4];
    Real depthCalibrations[keyframeCapacity,4];
    Real opticalToBodyRotations[keyframeCapacity,dimension,dimension];
    Real cameraOriginsBody[keyframeCapacity,dimension];
    Real disparityNoises[keyframeCapacity];
    Real noiseReferenceFocals[keyframeCapacity];
    Real baselines[keyframeCapacity];
    Real bodyRotations[keyframeCapacity,dimension,dimension];
    Real bodyPositions[keyframeCapacity,dimension];
    Real poseCovariances[keyframeCapacity,poseDimension,poseDimension];
    Integer filterSourceRevisions[keyframeCapacity];
    Integer filterCorrectionRevisions[keyframeCapacity];
    Integer vocabularyVersions[keyframeCapacity];
    Real histograms[keyframeCapacity,wordCapacity];
  end Catalog;

  function Empty
    input Integer generation = 1;
    input Integer vocabularyVersion = 1;
    output Catalog state;
  protected
    Frame emptyFrame;
  algorithm
    state.generation := generation;
    state.vocabularyVersion := vocabularyVersion;
    state.nextId := 1;
    state.nextSlot := 1;
    state.lastEpoch := -1;
    state.lastTime := 0.0;
    // Copy one canonical frame into each array slice. This initializes every
    // field once, including inactive payload, without a separate catalog-sized
    // zero initializer followed by overwriting each frame's metadata.
    emptyFrame := EmptyFrame();
    for slot in 1:keyframeCapacity loop
      state.occupied[slot] := false;
      state.generations[slot] := emptyFrame.generation;
      state.ids[slot] := emptyFrame.id;
      state.epochs[slot] := emptyFrame.epoch;
      state.imageTimes[slot] := emptyFrame.imageTime;
      state.counts[slot] := emptyFrame.count;
      state.featureEnabled[slot,:] := emptyFrame.enabled;
      state.descriptors[slot,:,:] := emptyFrame.descriptor;
      state.opticalPoints[slot,:,:] := emptyFrame.opticalPoint;
      state.pixelCoordinates[slot,:,:] := emptyFrame.pixels;
      state.rgbSizes[slot,:] := emptyFrame.rgbSize;
      state.depthSizes[slot,:] := emptyFrame.depthSize;
      state.rgbCalibrations[slot,:] := emptyFrame.rgbCalibration;
      state.depthCalibrations[slot,:] := emptyFrame.depthCalibration;
      state.opticalToBodyRotations[slot,:,:] := emptyFrame.opticalToBody;
      state.cameraOriginsBody[slot,:] := emptyFrame.cameraOriginBody;
      state.disparityNoises[slot] := emptyFrame.disparityNoise;
      state.noiseReferenceFocals[slot] := emptyFrame.noiseReferenceFx;
      state.baselines[slot] := emptyFrame.baseline;
      state.bodyRotations[slot,:,:] := emptyFrame.bodyRotation;
      state.bodyPositions[slot,:] := emptyFrame.bodyPosition;
      state.poseCovariances[slot,:,:] := emptyFrame.poseCovariance;
      state.filterSourceRevisions[slot] := emptyFrame.filterSourceRevision;
      state.filterCorrectionRevisions[slot] := emptyFrame.filterCorrectionRevision;
      state.vocabularyVersions[slot] := emptyFrame.vocabularyVersion;
      state.histograms[slot,:] := emptyFrame.histogram;
    end for;
  end Empty;


  // Internal slot extraction. Array domains are certified before reads.
  function ReadSlot
    input Catalog state;
    input Integer slot;
    output Frame frame;
  algorithm
    frame := EmptyFrame();
    if slot >= 1 and slot <= keyframeCapacity then
      frame.generation := state.generations[slot];
      frame.id := state.ids[slot];
      frame.epoch := state.epochs[slot];
      frame.imageTime := state.imageTimes[slot];
      frame.count := state.counts[slot];
      frame.enabled := state.featureEnabled[slot,:];
      frame.descriptor := state.descriptors[slot,:,:];
      frame.opticalPoint := state.opticalPoints[slot,:,:];
      frame.pixels := state.pixelCoordinates[slot,:,:];
      frame.rgbSize := state.rgbSizes[slot,:];
      frame.depthSize := state.depthSizes[slot,:];
      frame.rgbCalibration := state.rgbCalibrations[slot,:];
      frame.depthCalibration := state.depthCalibrations[slot,:];
      frame.opticalToBody := state.opticalToBodyRotations[slot,:,:];
      frame.cameraOriginBody := state.cameraOriginsBody[slot,:];
      frame.disparityNoise := state.disparityNoises[slot];
      frame.noiseReferenceFx := state.noiseReferenceFocals[slot];
      frame.baseline := state.baselines[slot];
      frame.bodyRotation := state.bodyRotations[slot,:,:];
      frame.bodyPosition := state.bodyPositions[slot,:];
      frame.poseCovariance := state.poseCovariances[slot,:,:];
      frame.filterSourceRevision := state.filterSourceRevisions[slot];
      frame.filterCorrectionRevision := state.filterCorrectionRevisions[slot];
      frame.vocabularyVersion := state.vocabularyVersions[slot];
      frame.histogram := state.histograms[slot,:];
    end if;
  end ReadSlot;

  function ValidCalibration
    input Real calibration[4];
    input Integer imageSize[2];
    output Boolean valid;
  algorithm
    valid := imageSize[1] > 0 and imageSize[1] <= 8192
      and imageSize[2] > 0 and imageSize[2] <= 8192
      and calibration[1] > 0 and calibration[1] <= 1e6
      and calibration[2] > 0 and calibration[2] <= 1e6
      and abs(calibration[3]) <= 8192 and abs(calibration[4]) <= 8192;
  end ValidCalibration;

  function ValidFrame
    input Frame frame;
    output Boolean valid;
  protected
    Integer measured;
    Real mass;
    Real norm;
    Real mean;
    Real inverseCovariance[poseDimension,poseDimension];
    Real scaledPivot;
    Boolean covarianceValid;
  algorithm
    valid := frame.generation >= 1 and frame.generation <= identifierLimit
      and frame.id >= 1 and frame.id < identifierLimit
      and frame.epoch >= 0 and frame.epoch <= identifierLimit
      and frame.imageTime >= 0 and frame.imageTime <= 1e9
      and frame.count >= 0 and frame.count <= featureCapacity
      and frame.vocabularyVersion >= 1 and frame.vocabularyVersion <= identifierLimit
      and ValidCalibration(frame.rgbCalibration,frame.rgbSize)
      and ValidCalibration(frame.depthCalibration,frame.depthSize)
      and frame.disparityNoise > 0 and frame.disparityNoise <= 100
      and frame.noiseReferenceFx > 0 and frame.noiseReferenceFx <= 1e6
      and frame.baseline > 0 and frame.baseline <= 10
      and RGBDUncertaintyProper(frame.opticalToBody)
      and RGBDUncertaintyProper(frame.bodyRotation);
    for axis in 1:dimension loop
      valid := valid and abs(frame.bodyPosition[axis]) <= 1e6
        and abs(frame.cameraOriginBody[axis]) <= 1e6;
    end for;
    (inverseCovariance,covarianceValid,scaledPivot) :=
      RGBDUncertaintyInverse6(frame.poseCovariance,1e-10);
    valid := valid and covarianceValid
      and frame.filterSourceRevision >= 0 and frame.filterSourceRevision < identifierLimit
      and frame.filterCorrectionRevision >= 0 and frame.filterCorrectionRevision < identifierLimit;
    measured := 0;
    for feature in 1:featureCapacity loop
      if frame.enabled[feature] then
        measured := measured+1;
        valid := valid and feature <= frame.count
          and frame.pixels[feature,1] >= 0 and frame.pixels[feature,1] < frame.rgbSize[2]
          and frame.pixels[feature,2] >= 0 and frame.pixels[feature,2] < frame.rgbSize[1]
          and frame.opticalPoint[feature,3] > 0;
        for axis in 1:dimension loop
          valid := valid and abs(frame.opticalPoint[feature,axis]) <= 1e6;
        end for;
        norm := 0.0;
        mean := 0.0;
        for sample in 1:descriptorSize loop
          valid := valid and abs(frame.descriptor[feature,sample]) <= 1.000001;
          norm := norm+frame.descriptor[feature,sample]^2;
          mean := mean+frame.descriptor[feature,sample];
        end for;
        valid := valid and abs(norm-1.0) <= 1e-6 and abs(mean) <= 1e-6;
      end if;
    end for;
    valid := valid and measured >= 8;
    mass := 0.0;
    for word in 1:wordCapacity loop
      valid := valid and frame.histogram[word] >= 0 and frame.histogram[word] <= 1;
      mass := mass+frame.histogram[word];
    end for;
    valid := valid and abs(mass-1.0) <= 1e-6;
  end ValidFrame;

  // Header checks are cheap relative to payload validation; full payload
  // validation belongs to admission or restore. Store operates on an already
  // validated, Modelica-owned catalog. Slots are not a function of ids: ids
  // are unique, ordered like epochs and image times, and the slot due for
  // eviction (nextSlot) is free while filling and outside the newest
  // retainedNewest ids once full.
  function ValidHeader "Header and identity invariants of a keyframe catalog"
    input Catalog state;
    output Boolean valid;
  protected
    Integer expectedOccupied;
    Integer actualOccupied;
    Integer newestSlot;
    Integer recentCount;
    Boolean identityValid;
  algorithm
    expectedOccupied := 0; actualOccupied := 0;
    newestSlot := 0; recentCount := 0; identityValid := false;
    valid := state.generation >= 1 and state.generation <= identifierLimit
      and state.vocabularyVersion >= 1 and state.vocabularyVersion <= identifierLimit
      and state.nextId >= 1 and state.nextId <= identifierLimit
      and state.nextSlot >= 1 and state.nextSlot <= keyframeCapacity
      and state.lastEpoch >= -1 and state.lastEpoch <= identifierLimit
      and state.lastTime >= 0 and state.lastTime <= 1e9;
    // Certify Integer domains before subtraction or computing array indices.
    if valid then
      expectedOccupied := min(state.nextId-1,keyframeCapacity);
      for slot in 1:keyframeCapacity loop
        if state.occupied[slot] then
          actualOccupied := actualOccupied+1;
          identityValid := state.ids[slot] >= 1 and state.ids[slot] < state.nextId;
          valid := valid and identityValid
            and state.generations[slot] == state.generation
            and state.vocabularyVersions[slot] == state.vocabularyVersion
            and state.epochs[slot] >= 0 and state.epochs[slot] <= state.lastEpoch
            and state.imageTimes[slot] >= 0 and state.imageTimes[slot] <= state.lastTime;
          if identityValid then
            if state.ids[slot] == state.nextId-1 then newestSlot := slot; end if;
            if state.ids[slot] >= state.nextId-retainedNewest then recentCount := recentCount+1; end if;
          end if;
          // Unique ids; a smaller id is an older capture. Replaces the former
          // contiguity chain, which no longer holds once places are thinned.
          for other in 1:slot-1 loop
            if state.occupied[other] then
              valid := valid and state.ids[other] <> state.ids[slot]
                and (if state.ids[other] < state.ids[slot]
                  then state.epochs[other] < state.epochs[slot] and state.imageTimes[other] < state.imageTimes[slot]
                  else state.epochs[other] > state.epochs[slot] and state.imageTimes[other] > state.imageTimes[slot]);
            end if;
          end for;
        end if;
      end for;
      valid := valid and actualOccupied == expectedOccupied
        and recentCount == min(state.nextId-1,retainedNewest)
        and ((state.nextId == 1 and state.lastEpoch == -1 and state.lastTime == 0)
          or (state.nextId > 1 and state.lastEpoch >= 0));
      if state.nextId > 1 then
        valid := valid and newestSlot >= 1;
        if newestSlot >= 1 then
          valid := valid and state.epochs[newestSlot] == state.lastEpoch
            and state.imageTimes[newestSlot] == state.lastTime;
        end if;
      end if;
      if actualOccupied < keyframeCapacity then
        valid := valid and not state.occupied[state.nextSlot];
      else
        valid := valid and state.occupied[state.nextSlot]
          and state.ids[state.nextSlot] < state.nextId-retainedNewest;
      end if;
    end if;
  end ValidHeader;

  function ValidCatalog
    input Catalog state;
    output Boolean valid;
  algorithm
    valid := ValidHeader(state);
    for slot in 1:keyframeCapacity loop
      if state.occupied[slot] then
        valid := valid and ValidFrame(ReadSlot(state,slot));
      end if;
    end for;
  end ValidCatalog;

  function Store "Admit one keyframe into the slot chosen by the previous Store"
    input Catalog previous;
    input Frame candidate;
    input Boolean requested;
    input Integer retention[keyframeCapacity] = fill(retentionFirst,keyframeCapacity)
      "Graph-side eviction tier per slot (retentionHeld/First/Bridged); the default suits catalogs without a graph";
    output Catalog next;
    output Boolean accepted;
    output Integer storedSlot;
    output Integer evictedId "Invalidate this generation/id in graph and landmark owners";
    output Integer rejectionReason "0 stored; 1 idle; 2 header or candidate; 3 retention holds the slot due for eviction";
  protected
    Boolean evictable;
  algorithm
    next := previous;
    accepted := false;
    storedSlot := 0;
    evictedId := 0;
    rejectionReason := if requested then 2 else 1;
    if requested and ValidHeader(previous) and previous.nextId < identifierLimit then
      accepted := candidate.generation == previous.generation
        and candidate.vocabularyVersion == previous.vocabularyVersion
        and candidate.id == previous.nextId and candidate.epoch > previous.lastEpoch
        and (previous.nextId == 1 or candidate.imageTime > previous.lastTime)
        and ValidFrame(candidate);
      // The victim was chosen by the previous Store. Refuse, with a distinct
      // reason, when the graph now holds it instead of orphaning its edges.
      if accepted and previous.occupied[previous.nextSlot]
        and retention[previous.nextSlot] <> retentionFirst and retention[previous.nextSlot] <> retentionBridged then
        accepted := false; rejectionReason := storeRetentionRefused;
      end if;
      if accepted then
        rejectionReason := 0;
        storedSlot := previous.nextSlot;
        evictedId := if previous.occupied[storedSlot] then previous.ids[storedSlot] else 0;
        next.generations[storedSlot] := candidate.generation;
        next.ids[storedSlot] := candidate.id;
        next.epochs[storedSlot] := candidate.epoch;
        next.imageTimes[storedSlot] := candidate.imageTime;
        next.counts[storedSlot] := candidate.count;
        next.featureEnabled[storedSlot,:] := candidate.enabled;
        next.descriptors[storedSlot,:,:] := candidate.descriptor;
        next.opticalPoints[storedSlot,:,:] := candidate.opticalPoint;
        next.pixelCoordinates[storedSlot,:,:] := candidate.pixels;
        next.rgbSizes[storedSlot,:] := candidate.rgbSize;
        next.depthSizes[storedSlot,:] := candidate.depthSize;
        next.rgbCalibrations[storedSlot,:] := candidate.rgbCalibration;
        next.depthCalibrations[storedSlot,:] := candidate.depthCalibration;
        next.opticalToBodyRotations[storedSlot,:,:] := candidate.opticalToBody;
        next.cameraOriginsBody[storedSlot,:] := candidate.cameraOriginBody;
        next.disparityNoises[storedSlot] := candidate.disparityNoise;
        next.noiseReferenceFocals[storedSlot] := candidate.noiseReferenceFx;
        next.baselines[storedSlot] := candidate.baseline;
        next.bodyRotations[storedSlot,:,:] := candidate.bodyRotation;
        next.bodyPositions[storedSlot,:] := candidate.bodyPosition;
        next.poseCovariances[storedSlot,:,:] := candidate.poseCovariance;
        next.filterSourceRevisions[storedSlot] := candidate.filterSourceRevision;
        next.filterCorrectionRevisions[storedSlot] := candidate.filterCorrectionRevision;
        next.vocabularyVersions[storedSlot] := candidate.vocabularyVersion;
        next.histograms[storedSlot,:] := candidate.histogram;
        next.occupied[storedSlot] := true;
        next.nextId := previous.nextId+1;
        // Choose the next victim now, so retrieval, loop verification and the
        // graph owners keep excluding exactly the slot the next Store reuses.
        (next.nextSlot,evictable) := ChooseVictim(next,retention);
        next.lastEpoch := candidate.epoch;
        next.lastTime := candidate.imageTime;
      end if;
    end if;
  end Store;

  function Lookup "Retained keyframe by generation and id"
    input Catalog state;
    input Integer generation;
    input Integer id;
    output Frame frame;
    output Boolean found;
    output Integer slot;
  algorithm
    frame := EmptyFrame();
    found := false;
    slot := 0;
    if generation == state.generation and id >= 1 and id < state.nextId and ValidHeader(state) then
      slot := SlotOf(state,id);
      if slot >= 1 then
        frame := ReadSlot(state,slot);
        found := true;
      end if;
    end if;
  end Lookup;

  function SlotOf "Slot holding a retained id, or 0 when the id is not retained"
    input Catalog state;
    input Integer id;
    output Integer slot;
  algorithm
    slot := 0;
    for candidate in 1:keyframeCapacity loop
      if slot == 0 and state.occupied[candidate] and state.ids[candidate] == id then
        slot := candidate;
      end if;
    end for;
  end SlotOf;

  function NewestSlot "Slot of the newest keyframe (id nextId-1), or 0 when empty"
    input Catalog state;
    output Integer slot;
  algorithm
    slot := if state.nextId > 1 then SlotOf(state,state.nextId-1) else 0;
  end NewestSlot;

  function OldestId "Smallest retained id, or nextId when the catalog is empty"
    input Catalog state;
    output Integer id;
  algorithm
    id := state.nextId;
    for slot in 1:keyframeCapacity loop
      if state.occupied[slot] and state.ids[slot] < id then id := state.ids[slot]; end if;
    end for;
  end OldestId;

  function IdOrder "Occupied slots in ascending id order"
    input Catalog state;
    output Integer order[keyframeCapacity] "order[rank] is the slot of the rank-th oldest retained id; 0 past count";
    output Integer count;
  protected
    Integer rank;
  algorithm
    // Pairwise ranks: O(keyframeCapacity^2) integer comparisons, no state.
    // Ids are unique in a valid header, so ranks form a permutation.
    order := fill(0,keyframeCapacity); count := 0;
    for slot in 1:keyframeCapacity loop
      if state.occupied[slot] then
        count := count+1; rank := 1;
        for other in 1:keyframeCapacity loop
          if state.occupied[other] and state.ids[other] < state.ids[slot] then rank := rank+1; end if;
        end for;
        if rank <= keyframeCapacity then order[rank] := slot; end if;
      end if;
    end for;
  end IdOrder;

  function ChooseVictim "Slot the next Store overwrites: a free slot, else the retained place whose loss leaves the shortest gap"
    input Catalog state;
    input Integer retention[keyframeCapacity];
    output Integer slot;
    output Boolean evictable "False when retention holds every candidate; the next Store then refuses";
  protected
    Integer order[keyframeCapacity]; Integer count; Integer node; Integer before; Integer tier;
    Integer bestTier; Integer fallback; Real gap; Real bestGap; Real fallbackGap; Real nearest; Real distance;
    Real midpoint[dimension]; Real nearestMidpoint; Real midpointDistance;
  algorithm
    slot := 0; evictable := true;
    for candidate in 1:keyframeCapacity loop
      if slot == 0 and not state.occupied[candidate] then slot := candidate; end if;
    end for;
    if slot == 0 then
      // Candidates exclude id 1 and the newest retainedNewest ids. The key is
      // (retention tier, spacing, id): the lower tier first, then the smallest
      // spacing its eviction leaves, then the older id. That spacing is the
      // cover metric: the larger of the distance from the keyframe to the
      // nearest other retained keyframe (the place stays covered) and the
      // distance from the midpoint of its retained id-order neighbours to the
      // nearest other retained keyframe (the path hole stays covered). Measuring
      // the hole by coverage, not by the neighbours' separation, keeps a folded
      // path (out and back) from losing its far end. Ranks run in ascending id with strict
      // comparisons, so equal keys keep the older id; ids are unique, so the
      // final lower-slot rule never has to act. The oldest retained id (rank
      // 1, only once id 1 is gone, e.g. a catalog restored from a ring-era
      // store) has no older neighbour; its path term is the full spacing to
      // the next retained id. Cost: IdOrder's 64^2 integer comparisons plus
      // 2*64^2 squared distances.
      (order,count) := IdOrder(state);
      bestTier := 0; bestGap := 0.0; fallback := 0; fallbackGap := 0.0;
      for rank in 1:keyframeCapacity-1 loop
        if rank < count then
          node := order[rank]; before := if rank > 1 then order[max(rank-1,1)] else node;
          if state.ids[node] <> 1 and state.ids[node] < state.nextId-retainedNewest then
            // Squared cover metric; rank 1 has no older neighbour, so its hole
            // term is the keyframe itself.
            for axis in 1:dimension loop
              midpoint[axis] := if rank > 1 then 0.5*(state.bodyPositions[before,axis]+state.bodyPositions[order[rank+1],axis])
                else state.bodyPositions[node,axis];
            end for;
            nearest := -1.0; nearestMidpoint := -1.0;
            for other in 1:keyframeCapacity loop
              if other <> node and state.occupied[other] then
                distance := 0.0; midpointDistance := 0.0;
                for axis in 1:dimension loop
                  distance := distance+(state.bodyPositions[other,axis]-state.bodyPositions[node,axis])^2;
                  midpointDistance := midpointDistance+(state.bodyPositions[other,axis]-midpoint[axis])^2;
                end for;
                if nearest < 0.0 or distance < nearest then nearest := distance; end if;
                if nearestMidpoint < 0.0 or midpointDistance < nearestMidpoint then nearestMidpoint := midpointDistance; end if;
              end if;
            end for;
            gap := max(nearest,nearestMidpoint);
            tier := retention[node];
            if (tier == retentionFirst or tier == retentionBridged)
              and (bestTier == 0 or tier < bestTier or (tier == bestTier and gap < bestGap)) then
              slot := node; bestTier := tier; bestGap := gap;
            end if;
            if fallback == 0 or gap < fallbackGap then fallback := node; fallbackGap := gap; end if;
          end if;
        end if;
      end for;
      if slot == 0 then
        // Keep a valid header; Store refuses this slot while retention holds it.
        slot := max(1,fallback); evictable := false;
      end if;
    end if;
  end ChooseVictim;

  function Reset
    input Catalog previous;
    input Integer generation;
    input Integer vocabularyVersion = 1;
    output Catalog next;
    output Boolean accepted;
  algorithm
    next := previous;
    accepted := generation >= 1 and generation > previous.generation and generation <= identifierLimit
      and vocabularyVersion >= 1 and vocabularyVersion <= identifierLimit;
    if accepted then
      next := Empty(generation,vocabularyVersion);
    end if;
  end Reset;
end RGBDKeyframes;
