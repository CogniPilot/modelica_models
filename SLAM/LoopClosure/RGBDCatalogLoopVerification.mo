within SLAM.LoopClosure;
// Geometric proposals use the same immutable catalog snapshot as retrieval.
// Graph admission and shared-image correlation remain separate owners.
package RGBDCatalogLoopVerification
  import RegistrationWhitenedResidual3 = Vision.Registration.RegistrationWhitenedResidual3;
  import RGBDUncertaintySkew = SLAM.Localization.RGBDUncertaintySkew;

  function PoseConsistent
    input Real referencePosition[3]; input Real currentPosition[3];
    input Real referenceRotation[3,3];
    input Real referenceCovariance[6,6]; input Real currentCovariance[6,6];
    input Real measuredTranslation[3]; input Real measuredCovariance[3,3];
    input Real maximumSquared = 9.0; input Real minimumPivot = 1e-10;
    input Real maximumTranslationInnovation = 2.0 "Motion disagreement limit; explicit relocalization policy may enlarge it";
    output Boolean accepted; output Real squared;
  protected
    Real residual[3]; Real arm[3,3]; Real covariance[3,3]; Boolean valid;
  algorithm
    residual := currentPosition-referencePosition-referenceRotation*measuredTranslation;
    arm := referenceRotation*RGBDUncertaintySkew(measuredTranslation);
    // A conservative screening bound, not an independent measurement update.
    // Four times the marginal sum bounds arbitrary correlations among the
    // two positions, retained-reference angle and measured translation.
    covariance := 4.0*(referenceCovariance[1:3,1:3]+currentCovariance[1:3,1:3]
      +LinearAlgebra.transformCovariance(referenceRotation,measuredCovariance)
      +LinearAlgebra.transformCovariance(arm,referenceCovariance[4:6,4:6]));
    (valid,squared) := RegistrationWhitenedResidual3(residual,covariance,minimumPivot);
    accepted := valid and maximumSquared > 0.0 and squared <= maximumSquared
      and maximumTranslationInnovation > 0.0
      and residual*residual <= maximumTranslationInnovation*maximumTranslationInnovation;
  end PoseConsistent;
  import RGBDKeyframeRetrieval = SLAM.LoopClosure.RGBDKeyframeRetrieval;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDLoopVerification = SLAM.LoopClosure.RGBDLoopVerification;

  function VerifyCandidate
    input RGBDKeyframes.Catalog catalog;
    input RGBDKeyframes.Frame current;
    input Integer candidateId; input Integer candidateSlot;
    input Real appearanceScore; input Boolean requested;
    input Integer seed = 7; input Integer trials = 96; input Integer refinements = 4;
    input Integer minimumInliers = 12; input Real minimumFraction = 0.5;
    input Real inlierDistance = 0.08; input Real maximumRms = 0.03;
    input Real minimumAge = 5.0; input Real minimumSimilarity = 0.35;
    input Real descriptorRatio = 0.8; input Real maximumDescriptorDistance = 0.8;
    input Real coordinateLimit = 100.0; input Real rankTolerance = 1e-8;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
    output RGBDLoopVerification.Proposal proposal;
    input Boolean calibratedResiduals = true;
    input Real maximumNormalizedSquared = 9.0;
    input Boolean poseConsistentLoops = false;
    input Real maximumPoseInnovation = 2.0;
    input Real poseConsistencySquared = 9.0;
  protected
    RGBDKeyframes.Frame reference;
    Boolean bound; Boolean poseConsistent; Real poseSquared;
  algorithm
    proposal := RGBDLoopVerification.EmptyProposal(seed);
    if requested then
      // Check domains before reading a slot. Capture will replace nextSlot,
      // so that identity cannot supply a retained loop endpoint.
      bound := RGBDKeyframes.ValidHeader(catalog)
        and candidateSlot >= 1 and candidateSlot <= RGBDKeyframes.keyframeCapacity
        and candidateSlot <> catalog.nextSlot and candidateId >= 1 and candidateId < catalog.nextId
        and current.generation == catalog.generation
        and current.vocabularyVersion == catalog.vocabularyVersion
        and current.id == catalog.nextId and current.epoch > catalog.lastEpoch
        and (catalog.nextId == 1 or current.imageTime > catalog.lastTime)
        and minimumSimilarity >= 0 and minimumSimilarity <= 1
        and appearanceScore >= minimumSimilarity and appearanceScore <= 1;
      proposal.rejectionReason := 10 "Catalog/appearance identity binding refused";
      if bound then
        bound := catalog.occupied[candidateSlot] and catalog.ids[candidateSlot] == candidateId
          and catalog.generations[candidateSlot] == current.generation
          and catalog.vocabularyVersions[candidateSlot] == current.vocabularyVersion;
        if bound then
          reference := RGBDKeyframes.ReadSlot(catalog,candidateSlot);
          proposal := RGBDLoopVerification.Verify(reference,current,true,seed,trials,refinements,
            minimumInliers,minimumFraction,inlierDistance,maximumRms,minimumAge,descriptorRatio,
            maximumDescriptorDistance,coordinateLimit,rankTolerance,localizationSigma,depthInflation,minimumPivot,
            calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared);
          if proposal.verified and poseConsistentLoops then
            (poseConsistent,poseSquared) := PoseConsistent(reference.bodyPosition,current.bodyPosition,
              reference.bodyRotation,reference.poseCovariance,current.poseCovariance,
              proposal.bodyTranslation,proposal.covariance[1:3,1:3],minimumPivot=minimumPivot,
              maximumSquared=poseConsistencySquared,maximumTranslationInnovation=maximumPoseInnovation);
            if not poseConsistent then
              proposal.verified := false; proposal.rejectionReason := 11;
            end if;
          end if;
        end if;
      end if;
    end if;
  end VerifyCandidate;

  record Batch
    RGBDKeyframes.Frame prepared;
    Boolean retrievalAccepted; Integer retrievalRejectionReason;
    Real wordIndex[RGBDKeyframes.featureCapacity];
    Integer candidateId[RGBDKeyframeRetrieval.proposalCapacity];
    Integer candidateSlot[RGBDKeyframeRetrieval.proposalCapacity];
    Real candidateScore[RGBDKeyframeRetrieval.proposalCapacity];
    Integer candidateCount; Real assignmentCount;
    RGBDLoopVerification.Proposal proposals[RGBDKeyframeRetrieval.proposalCapacity];
    Integer verifiedCount;
    Integer nextSeeds[RGBDKeyframeRetrieval.proposalCapacity];
  end Batch;

  function ProposeCapture
    input RGBDKeyframes.Catalog catalog;
    input RGBDKeyframes.Frame measurement;
    input Real vocabulary[RGBDKeyframes.wordCapacity,RGBDKeyframes.descriptorSize];
    input Real vocabularyEnabled[RGBDKeyframes.wordCapacity];
    input Boolean requested;
    input Integer seeds[RGBDKeyframeRetrieval.proposalCapacity];
    input Real maximumWordDistanceSquared = 0.8; input Integer minimumAssignments = 8;
    input Real minimumSimilarity = 0.35; input Real minimumAge = 5.0;
    input Integer trials = 96; input Integer refinements = 4;
    input Integer minimumInliers = 12; input Real minimumFraction = 0.5;
    input Real inlierDistance = 0.08; input Real maximumRms = 0.03;
    input Real descriptorRatio = 0.8; input Real maximumDescriptorDistance = 0.8;
    input Real coordinateLimit = 100.0; input Real rankTolerance = 1e-8;
    input Real localizationSigma = 0.5; input Real depthInflation = 1.0; input Real minimumPivot = 1e-10;
    output Batch result;
    input Boolean calibratedResiduals = true;
    input Real maximumNormalizedSquared = 9.0;
    input Boolean poseConsistentLoops = false;
    input Real maximumPoseInnovation = 2.0;
    input Real poseConsistencySquared = 9.0;
  algorithm
    (result.prepared,result.retrievalAccepted,result.retrievalRejectionReason,result.wordIndex,result.candidateId,result.candidateSlot,result.candidateScore,
      result.candidateCount,result.assignmentCount) := RGBDKeyframeRetrieval.PrepareCapture(catalog,measurement,
        vocabulary,vocabularyEnabled,requested,maximumWordDistanceSquared,minimumAssignments,minimumSimilarity,minimumAge);
    result.verifiedCount := 0;
    for rank in 1:RGBDKeyframeRetrieval.proposalCapacity loop
      result.proposals[rank] := VerifyCandidate(catalog,result.prepared,result.candidateId[rank],result.candidateSlot[rank],result.candidateScore[rank],
        result.retrievalAccepted and rank <= result.candidateCount,seeds[rank],trials,refinements,minimumInliers,
        minimumFraction,inlierDistance,maximumRms,minimumAge,minimumSimilarity,descriptorRatio,
        maximumDescriptorDistance,coordinateLimit,rankTolerance,localizationSigma,depthInflation,minimumPivot,
        calibratedResiduals=calibratedResiduals,maximumNormalizedSquared=maximumNormalizedSquared,
        poseConsistentLoops=poseConsistentLoops,maximumPoseInnovation=maximumPoseInnovation,poseConsistencySquared=poseConsistencySquared);
      result.nextSeeds[rank] := result.proposals[rank].nextSeed;
      result.verifiedCount := result.verifiedCount+(if result.proposals[rank].verified then 1 else 0);
    end for;
  end ProposeCapture;
end RGBDCatalogLoopVerification;
