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
  constant Integer keyframeCapacity = 128;
  constant Integer dimension = 3;
  constant Integer poseDimension = 6;
  constant Integer identifierLimit = 1000000000;

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
    valid := valid and covarianceValid;
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

  // Header checks are cheap; full payload validation belongs to admission or
  // restore. Store operates on an already validated, Modelica-owned catalog.
  function ValidHeader
    input Catalog state;
    output Boolean valid;
  protected
    Integer expectedOccupied;
    Integer actualOccupied;
    Integer olderSlot;
    Integer newestSlot;
    Boolean identityValid;
  algorithm
    expectedOccupied := 0; actualOccupied := 0;
    olderSlot := 1; newestSlot := 1; identityValid := false;
    valid := state.generation >= 1 and state.generation <= identifierLimit
      and state.vocabularyVersion >= 1 and state.vocabularyVersion <= identifierLimit
      and state.nextId >= 1 and state.nextId <= identifierLimit
      and state.nextSlot >= 1 and state.nextSlot <= keyframeCapacity
      and state.lastEpoch >= -1 and state.lastEpoch <= identifierLimit
      and state.lastTime >= 0 and state.lastTime <= 1e9;
    // Certify Integer domains before subtraction or computing array indices.
    if valid then
      valid := state.nextSlot == mod(state.nextId-1,keyframeCapacity)+1;
      expectedOccupied := min(state.nextId-1,keyframeCapacity);
      for slot in 1:keyframeCapacity loop
        if state.occupied[slot] then
          actualOccupied := actualOccupied+1;
          identityValid := state.ids[slot] >= max(1,state.nextId-keyframeCapacity)
            and state.ids[slot] < state.nextId;
          valid := valid and identityValid
            and state.generations[slot] == state.generation
            and state.vocabularyVersions[slot] == state.vocabularyVersion
            and state.epochs[slot] >= 0 and state.epochs[slot] <= state.lastEpoch
            and state.imageTimes[slot] >= 0 and state.imageTimes[slot] <= state.lastTime;
          if identityValid then
            valid := valid and mod(state.ids[slot]-1,keyframeCapacity)+1 == slot;
            if state.ids[slot] > max(1,state.nextId-keyframeCapacity) then
              olderSlot := mod(state.ids[slot]-2,keyframeCapacity)+1;
              valid := valid and state.occupied[olderSlot]
                and state.ids[olderSlot] == state.ids[slot]-1
                and state.epochs[olderSlot] < state.epochs[slot]
                and state.imageTimes[olderSlot] < state.imageTimes[slot];
            end if;
          end if;
        end if;
      end for;
      valid := valid and actualOccupied == expectedOccupied
        and ((state.nextId == 1 and state.lastEpoch == -1 and state.lastTime == 0)
          or (state.nextId > 1 and state.lastEpoch >= 0));
      if state.nextId > 1 then
        newestSlot := mod(state.nextId-2,keyframeCapacity)+1;
        valid := valid and state.occupied[newestSlot]
          and state.ids[newestSlot] == state.nextId-1
          and state.epochs[newestSlot] == state.lastEpoch
          and state.imageTimes[newestSlot] == state.lastTime;
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

  function Store
    input Catalog previous;
    input Frame candidate;
    input Boolean requested;
    output Catalog next;
    output Boolean accepted;
    output Integer storedSlot;
    output Integer evictedId "Invalidate this generation/id in graph and landmark owners";
  algorithm
    next := previous;
    accepted := false;
    storedSlot := 0;
    evictedId := 0;
    if requested and ValidHeader(previous) and previous.nextId < identifierLimit then
      accepted := candidate.generation == previous.generation
        and candidate.vocabularyVersion == previous.vocabularyVersion
        and candidate.id == previous.nextId and candidate.epoch > previous.lastEpoch
        and (previous.nextId == 1 or candidate.imageTime > previous.lastTime)
        and ValidFrame(candidate);
      if accepted then
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
        next.vocabularyVersions[storedSlot] := candidate.vocabularyVersion;
        next.histograms[storedSlot,:] := candidate.histogram;
        next.occupied[storedSlot] := true;
        next.nextId := previous.nextId+1;
        next.nextSlot := mod(previous.nextSlot,keyframeCapacity)+1;
        next.lastEpoch := candidate.epoch;
        next.lastTime := candidate.imageTime;
      end if;
    end if;
  end Store;

  function Lookup
    input Catalog state;
    input Integer generation;
    input Integer id;
    output Frame frame;
    output Boolean found;
    output Integer slot;
  protected
    Integer proposedSlot;
  algorithm
    frame := EmptyFrame();
    found := false;
    slot := 0;
    if generation == state.generation and id >= 1 and id < state.nextId and ValidHeader(state) then
      proposedSlot := mod(id-1,keyframeCapacity)+1;
      if state.occupied[proposedSlot] and state.ids[proposedSlot] == id then
        frame := ReadSlot(state,proposedSlot);
        found := true;
        slot := proposedSlot;
      end if;
    end if;
  end Lookup;

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
