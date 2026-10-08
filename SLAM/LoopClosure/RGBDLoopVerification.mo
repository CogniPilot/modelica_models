within SLAM.LoopClosure;
// Geometric verification of a retrieved keyframe. A verified measurement is a
// proposal: graph identity/eviction and shared-image correlation policies must
// still admit it before any optimizer or estimator state can change.
package RGBDLoopVerification
  import FitRigidPointPairs = Vision.Registration.FitRigidPointPairs;
  import MatchRGBDDescriptors = Vision.Matching.MatchRGBDDescriptors;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDOpticalPointCovariance = SLAM.Localization.RGBDOpticalPointCovariance;
  import RGBDOpticalToBodyEdge = SLAM.LoopClosure.RGBDOpticalToBodyEdge;
  import RGBDRegistrationSandwich = SLAM.Localization.RGBDRegistrationSandwich;
  import RegistrationPairResidual = Vision.Registration.RegistrationPairResidual;
  import RegistrationWhitenedResidual3 = Vision.Registration.RegistrationWhitenedResidual3;

  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer dimension = 3;
  constant Integer poseDimension = 6;
  constant Integer maximumTrials = 256;
  constant Integer maximumRefinements = 8;
  constant Integer randomModulus = 2147483647;

  // Park-Miller with Schrage decomposition. Every Integer intermediate fits
  // signed 32-bit storage; no large floating-point RNG product is required.
  function NextSample
    input Integer previous;
    output Integer next;
  protected
    Integer high; Integer low; Integer difference;
  algorithm
    next := 0;
    if previous >= 1 and previous < randomModulus then
      high := div(previous,127773);
      low := mod(previous,127773);
      difference := 16807*low-2836*high;
      next := if difference > 0 then difference else difference+randomModulus;
    end if;
  end NextSample;

  function ScorePairs
    input Real sourcePoint[:,dimension];
    input Real targetPoint[size(sourcePoint,1),dimension];
    input Real pairEnabled[size(sourcePoint,1)];
    input Real rotation[dimension,dimension]; input Real translation[dimension];
    input Real distanceLimit;
    output Real mask[size(sourcePoint,1)];
    output Integer count;
    output Real cost;
    input Boolean calibratedResiduals = false;
    input Real sourceCovariance[size(sourcePoint,1),dimension,dimension] = zeros(size(sourcePoint,1),dimension,dimension);
    input Real targetCovariance[size(sourcePoint,1),dimension,dimension] = zeros(size(sourcePoint,1),dimension,dimension);
    input Real maximumNormalizedSquared = 9.0;
    input Real minimumPivot = 1e-10;
  protected
    Real residual[dimension]; Real squared; Boolean residualValid;
  algorithm
    mask := zeros(size(sourcePoint,1)); count := 0; cost := 0.0;
    for slot in 1:size(sourcePoint,1) loop
      if pairEnabled[slot] == 1.0 then
        if calibratedResiduals then
          (residualValid,squared) := RegistrationPairResidual(sourcePoint[slot,:],targetPoint[slot,:],
            rotation,translation,true,sourceCovariance[slot,:,:],targetCovariance[slot,:,:],minimumPivot);
        else
          residual := rotation*sourcePoint[slot,:]+translation-targetPoint[slot,:];
          squared := residual*residual; residualValid := squared >= 0.0;
        end if;
        if residualValid and squared <= (if calibratedResiduals then maximumNormalizedSquared else distanceLimit^2) then
          mask[slot] := 1.0; count := count+1; cost := cost+squared;
        end if;
      end if;
    end for;
  end ScorePairs;

  // Three-point hypotheses are scored against the entire original sparse
  // domain. The slot list is only a sampling index, never a prefix extent.
  // A finite trial budget can miss a real loop; it does not certify recall.
  function Consensus
    input Real sourcePoint[:,dimension];
    input Real targetPoint[size(sourcePoint,1),dimension];
    input Real pairEnabled[size(sourcePoint,1)];
    input Boolean requested;
    input Integer seed; input Integer trials; input Integer refinements;
    input Integer minimumInliers; input Real minimumFraction;
    input Real inlierDistance; input Real maximumRms;
    input Real coordinateLimit; input Real rankTolerance;
    output Boolean valid;
    output Integer rejectionReason;
    output Real rotation[dimension,dimension]; output Real translation[dimension];
    output Real inlierMask[size(sourcePoint,1)];
    output Integer matchCount; output Integer inlierCount;
    output Real rms; output Integer nextSeed;
    input Boolean calibratedResiduals = false;
    input Real sourceCovariance[size(sourcePoint,1),dimension,dimension] = zeros(size(sourcePoint,1),dimension,dimension);
    input Real targetCovariance[size(sourcePoint,1),dimension,dimension] = zeros(size(sourcePoint,1),dimension,dimension);
    input Real maximumNormalizedSquared = 9.0;
    input Real minimumPivot = 1e-10;
  protected
    Boolean configuration; Boolean payload; Boolean eligible; Boolean stable;
    Boolean covarianceValid;
    Integer slots[size(sourcePoint,1)]; Integer first; Integer second; Integer third;
    Integer chosen[dimension]; Integer state; Integer count; Integer bestCount;
    Real sampleSource[dimension,dimension]; Real sampleTarget[dimension,dimension];
    Real candidateRotation[dimension,dimension]; Real candidateTranslation[dimension];
    Real candidateMask[size(sourcePoint,1)]; Real bestMask[size(sourcePoint,1)];
    Real workingMask[size(sourcePoint,1)]; Real cost; Real bestCost;
    Real fitAccepted; Real fitReason; Real fitCount; Real invalidCount; Real rank;
    Real fitCost; Real fitRms; Real gap; Real sourceCentroid[dimension]; Real targetCentroid[dimension];
    Real unusedScore;
  algorithm
    valid := false; rejectionReason := 1;
    rotation := identity(dimension); translation := zeros(dimension);
    inlierMask := zeros(size(sourcePoint,1)); matchCount := 0; inlierCount := 0;
    rms := 0.0; nextSeed := seed;
    if requested then
      configuration := size(sourcePoint,1) >= dimension and size(sourcePoint,1) <= featureCapacity
        and seed >= 1 and seed < randomModulus and trials >= 1 and trials <= maximumTrials
        and refinements >= 1 and refinements <= maximumRefinements
        and minimumInliers >= dimension and minimumInliers <= size(sourcePoint,1)
        and minimumFraction > 0.0 and minimumFraction <= 1.0
        and inlierDistance > 0.0 and inlierDistance <= 10.0
        and maximumRms >= 0.0 and maximumRms <= inlierDistance
        and coordinateLimit > 0.0 and coordinateLimit <= 1e4
        and rankTolerance >= 1e-12 and rankTolerance <= 1e-2;
      if calibratedResiduals then
        configuration := configuration and maximumNormalizedSquared > 0 and maximumNormalizedSquared <= 1e6
          and minimumPivot > 0 and minimumPivot < 1;
      end if;
      rejectionReason := 2;
      if configuration then
        slots := fill(0,size(sourcePoint,1)); payload := true;
        for slot in 1:size(sourcePoint,1) loop
          payload := payload and (pairEnabled[slot] == 0.0 or pairEnabled[slot] == 1.0);
          if pairEnabled[slot] == 1.0 then
            matchCount := matchCount+1; slots[matchCount] := slot;
            for axis in 1:dimension loop
              payload := payload and abs(sourcePoint[slot,axis]) <= coordinateLimit
                and abs(targetPoint[slot,axis]) <= coordinateLimit;
            end for;
            if calibratedResiduals then
              (covarianceValid,unusedScore) := RegistrationWhitenedResidual3(zeros(dimension),sourceCovariance[slot,:,:],minimumPivot);
              payload := payload and covarianceValid;
              (covarianceValid,unusedScore) := RegistrationWhitenedResidual3(zeros(dimension),targetCovariance[slot,:,:],minimumPivot);
              payload := payload and covarianceValid;
            end if;
          end if;
        end for;
        rejectionReason := 3;
        if payload then
          rejectionReason := 4;
          if matchCount >= minimumInliers then
            bestCount := 0; bestCost := 1e100; bestMask := zeros(size(sourcePoint,1)); state := seed;
            for hypothesis in 1:trials loop
              state := NextSample(state); first := mod(state,matchCount)+1;
              state := NextSample(state); second := mod(state,matchCount-1)+1;
              if second >= first then second := second+1; end if;
              state := NextSample(state); third := mod(state,matchCount-2)+1;
              if third >= min(first,second) then third := third+1; end if;
              if third >= max(first,second) then third := third+1; end if;
              chosen := {slots[first],slots[second],slots[third]};
              for point in 1:dimension loop
                sampleSource[point,:] := sourcePoint[chosen[point],:];
                sampleTarget[point,:] := targetPoint[chosen[point],:];
              end for;
              (fitAccepted,fitReason,candidateRotation,candidateTranslation,fitCount,invalidCount,
                rank,fitCost,fitRms,gap,sourceCentroid,targetCentroid) := FitRigidPointPairs(
                  sampleSource,sampleTarget,ones(dimension),dimension,coordinateLimit,rankTolerance,
                  if calibratedResiduals then 1e6 else inlierDistance);
              if fitAccepted == 1.0 then
                (candidateMask,count,cost) := ScorePairs(sourcePoint,targetPoint,pairEnabled,
                  candidateRotation,candidateTranslation,inlierDistance,calibratedResiduals=calibratedResiduals,
                  sourceCovariance=sourceCovariance,targetCovariance=targetCovariance,
                  maximumNormalizedSquared=maximumNormalizedSquared,minimumPivot=minimumPivot);
                if count > bestCount or (count == bestCount and cost < bestCost) then
                  bestCount := count; bestCost := cost; bestMask := candidateMask;
                end if;
              end if;
            end for;
            nextSeed := state;
            eligible := bestCount >= minimumInliers and bestCount >= minimumFraction*matchCount;
            stable := false; workingMask := bestMask; rejectionReason := 5;
            // Stop doing fits after stability/refusal, while keeping a bounded
            // compact loop. The final fit and covariance must use one mask.
            for refinement in 1:refinements loop
              if eligible and not stable then
                (fitAccepted,fitReason,candidateRotation,candidateTranslation,fitCount,invalidCount,
                  rank,fitCost,fitRms,gap,sourceCentroid,targetCentroid) := FitRigidPointPairs(
                    sourcePoint,targetPoint,workingMask,size(sourcePoint,1),coordinateLimit,rankTolerance,
                    if calibratedResiduals then 1e6 else maximumRms);
                eligible := fitAccepted == 1.0;
                if not eligible then rejectionReason := 5; end if;
                if eligible then
                  (candidateMask,count,cost) := ScorePairs(sourcePoint,targetPoint,pairEnabled,
                    candidateRotation,candidateTranslation,inlierDistance,calibratedResiduals=calibratedResiduals,
                    sourceCovariance=sourceCovariance,targetCovariance=targetCovariance,
                    maximumNormalizedSquared=maximumNormalizedSquared,minimumPivot=minimumPivot);
                  eligible := count >= minimumInliers and count >= minimumFraction*matchCount;
                  stable := true;
                  for slot in 1:size(sourcePoint,1) loop
                    stable := stable and candidateMask[slot] == workingMask[slot];
                  end for;
                  if eligible and stable then
                    rotation := candidateRotation; translation := candidateTranslation;
                    inlierMask := workingMask; inlierCount := count; rms := fitRms;
                    valid := true; rejectionReason := 0;
                  else
                    workingMask := candidateMask;
                    rejectionReason := if eligible then 6 else 5;
                  end if;
                end if;
              end if;
            end for;
          end if;
        end if;
      end if;
    end if;
  end Consensus;

  record Proposal
    Boolean verified;
    Integer rejectionReason;
    Integer generation; Integer referenceId; Integer currentId;
    Integer referenceEpoch; Integer currentEpoch;
    Integer matchedCount; Integer inlierCount;
    Real rms; Integer nextSeed;
    Boolean inliers[featureCapacity];
    Integer partners[featureCapacity];
    Real opticalRotation[dimension,dimension]; Real opticalTranslation[dimension];
    Real bodyRotation[dimension,dimension]; Real bodyTranslation[dimension];
    Real covariance[poseDimension,poseDimension]; Real information[poseDimension,poseDimension];
  end Proposal;

  function EmptyProposal
    input Integer seed;
    output Proposal result;
  algorithm
    result.verified := false; result.rejectionReason := 1;
    result.generation := 0; result.referenceId := 0; result.currentId := 0;
    result.referenceEpoch := -1; result.currentEpoch := -1;
    result.matchedCount := 0; result.inlierCount := 0; result.rms := 0.0; result.nextSeed := seed;
    result.inliers := fill(false,featureCapacity); result.partners := fill(0,featureCapacity);
    result.opticalRotation := identity(dimension); result.opticalTranslation := zeros(dimension);
    result.bodyRotation := identity(dimension); result.bodyTranslation := zeros(dimension);
    result.covariance := zeros(poseDimension,poseDimension); result.information := zeros(poseDimension,poseDimension);
  end EmptyProposal;

  function Verify
    input RGBDKeyframes.Frame reference;
    input RGBDKeyframes.Frame current;
    input Boolean requested;
    input Integer seed = 7; input Integer trials = 96; input Integer refinements = 4;
    input Integer minimumInliers = 12; input Real minimumFraction = 0.5;
    input Real inlierDistance = 0.08; input Real maximumRms = 0.03;
    input Real minimumAge = 5.0; input Real descriptorRatio = 0.8;
    input Real maximumDescriptorDistance = 0.8;
    input Real coordinateLimit = 100.0; input Real rankTolerance = 1e-8;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
    output Proposal result;
    input Boolean calibratedResiduals = true;
    input Real maximumNormalizedSquared = 9.0;
  protected
    Boolean configuration; Boolean identityValid; Boolean consensusValid; Boolean edgeValid;
    Integer consensusReason; Integer edgeReason; Integer matches; Integer inliers; Integer nextSeed;
    Real referenceMask[featureCapacity]; Real currentMask[featureCapacity];
    Real partners[featureCapacity]; Real pairEnabled[featureCapacity];
    Real sourcePoint[featureCapacity,dimension]; Real targetPoint[featureCapacity,dimension];
    Real nearest[featureCapacity]; Real second[featureCapacity]; Real count; Real matchValid;
    Real invalidReference; Real invalidCurrent; Real mask[featureCapacity];
    Real C[dimension,dimension]; Real t[dimension]; Real rms;
    Real uncertaintyValid; Real uncertaintyReason; Real uncertaintyCount; Real uncertaintyInvalid;
    Real relativeCovariance[poseDimension,poseDimension]; Real unusedCovariance[poseDimension,poseDimension];
    Real normal[poseDimension,poseDimension]; Real noise[poseDimension,poseDimension]; Real J[poseDimension,poseDimension];
    Real pivot; Real D[dimension,dimension]; Real u[dimension];
    Real edgeCovariance[poseDimension,poseDimension]; Real information[poseDimension,poseDimension];
    Real sourceCovariance[featureCapacity,dimension,dimension]; Real targetCovariance[featureCapacity,dimension,dimension];
  algorithm
    result := EmptyProposal(seed);
    if requested then
      configuration := minimumAge >= 0.0 and minimumAge <= 1e6
        and seed >= 1 and seed < randomModulus and trials >= 1 and trials <= maximumTrials
        and refinements >= 1 and refinements <= maximumRefinements
        and minimumInliers >= dimension and minimumInliers <= featureCapacity
        and minimumFraction > 0.0 and minimumFraction <= 1.0
        and inlierDistance > 0.0 and inlierDistance <= 10.0
        and maximumRms >= 0.0 and maximumRms <= inlierDistance
        and coordinateLimit > 0.0 and coordinateLimit <= 1e4
        and rankTolerance >= 1e-12 and rankTolerance <= 1e-2
        and descriptorRatio > 0.0 and descriptorRatio < 1.0
        and maximumDescriptorDistance >= 0.0 and maximumDescriptorDistance <= 2.0
        and localizationSigma > 0.0 and localizationSigma <= 10.0
        and depthInflation >= 1.0 and depthInflation <= 100.0
        and minimumPivot > 0.0 and minimumPivot < 1.0;
      if calibratedResiduals then
        configuration := configuration and maximumNormalizedSquared > 0 and maximumNormalizedSquared <= 1e6;
      end if;
      result.rejectionReason := 2;
      if configuration then
        identityValid := reference.generation == current.generation and reference.id < current.id
          and reference.epoch < current.epoch and reference.imageTime < current.imageTime
          and current.imageTime-reference.imageTime >= minimumAge
          and reference.vocabularyVersion == current.vocabularyVersion;
        result.rejectionReason := 3;
        if identityValid then
          result.rejectionReason := 4;
          if RGBDKeyframes.ValidFrame(reference) and RGBDKeyframes.ValidFrame(current) then
            // The current sandwich model assumes a common disparity noise and
            // stereo baseline. Refuse incompatible snapshots rather than reuse
            // current calibration for the retained reference image.
            result.rejectionReason := 5;
            if reference.disparityNoise == current.disparityNoise and reference.baseline == current.baseline then
              for slot in 1:featureCapacity loop
                referenceMask[slot] := if reference.enabled[slot] then 1.0 else 0.0;
                currentMask[slot] := if current.enabled[slot] then 1.0 else 0.0;
              end for;
              (partners,pairEnabled,sourcePoint,targetPoint,count,matchValid,invalidReference,invalidCurrent,nearest,second) :=
                MatchRGBDDescriptors(reference.descriptor,current.descriptor,reference.opticalPoint,current.opticalPoint,
                  referenceMask,currentMask,reference.count,current.count,descriptorRatio,maximumDescriptorDistance,
                  0.0,identity(dimension),zeros(dimension),0.0);
              result.matchedCount := integer(count); result.rejectionReason := 6;
              if matchValid == 1.0 and invalidReference == 0.0 and invalidCurrent == 0.0 then
                sourceCovariance := zeros(featureCapacity,dimension,dimension);
                targetCovariance := zeros(featureCapacity,dimension,dimension);
                if calibratedResiduals then
                  for slot in 1:featureCapacity loop
                    if pairEnabled[slot] == 1.0 then
                      sourceCovariance[slot,:,:] := RGBDOpticalPointCovariance(sourcePoint[slot,:],
                        reference.rgbCalibration[1],reference.rgbCalibration[2],localizationSigma,
                        reference.disparityNoise,reference.noiseReferenceFx,reference.baseline,depthInflation);
                      targetCovariance[slot,:,:] := RGBDOpticalPointCovariance(targetPoint[slot,:],
                        current.rgbCalibration[1],current.rgbCalibration[2],localizationSigma,
                        current.disparityNoise,current.noiseReferenceFx,current.baseline,depthInflation);
                    end if;
                  end for;
                end if;
                (consensusValid,consensusReason,C,t,mask,matches,inliers,rms,nextSeed) := Consensus(
                  sourcePoint,targetPoint,pairEnabled,true,seed,trials,refinements,minimumInliers,minimumFraction,
                  inlierDistance,maximumRms,coordinateLimit,rankTolerance,
                  calibratedResiduals=calibratedResiduals,sourceCovariance=sourceCovariance,targetCovariance=targetCovariance,
                  maximumNormalizedSquared=maximumNormalizedSquared,minimumPivot=minimumPivot);
                result.nextSeed := nextSeed; result.rejectionReason := 7;
                if consensusValid then
                  (uncertaintyValid,uncertaintyReason,uncertaintyCount,uncertaintyInvalid,relativeCovariance,
                    unusedCovariance,normal,noise,J,pivot) := RGBDRegistrationSandwich(sourcePoint,targetPoint,
                      mask,featureCapacity,1.0,C,t,reference.bodyRotation,reference.opticalToBody,
                      reference.cameraOriginBody,reference.rgbCalibration[1:2],current.rgbCalibration[1:2],
                      reference.noiseReferenceFx,current.noiseReferenceFx,reference.baseline,localizationSigma,
                      reference.disparityNoise,depthInflation,coordinateLimit,minimumPivot);
                  result.rejectionReason := 8;
                  if uncertaintyValid == 1.0 then
                    (D,u,J,edgeCovariance,information,edgeValid,edgeReason,pivot) := RGBDOpticalToBodyEdge(C,t,
                      relativeCovariance,reference.opticalToBody,current.opticalToBody,
                      reference.cameraOriginBody,current.cameraOriginBody,true,coordinateLimit,minimumPivot);
                    result.rejectionReason := 9;
                    if edgeValid then
                      result.verified := true; result.rejectionReason := 0;
                      result.generation := current.generation;
                      result.referenceId := reference.id; result.currentId := current.id;
                      result.referenceEpoch := reference.epoch; result.currentEpoch := current.epoch;
                      result.inlierCount := inliers; result.rms := rms;
                      result.opticalRotation := C; result.opticalTranslation := t;
                      result.bodyRotation := D; result.bodyTranslation := u;
                      result.covariance := edgeCovariance; result.information := information;
                      for slot in 1:featureCapacity loop
                        result.inliers[slot] := mask[slot] == 1.0;
                        result.partners[slot] := if mask[slot] == 1.0 then integer(partners[slot]) else 0;
                      end for;
                    end if;
                  end if;
                end if;
              end if;
            end if;
          end if;
        end if;
      end if;
    end if;
  end Verify;
end RGBDLoopVerification;
