within SLAM.LoopClosure;
// Appearance queries share the retained geometry catalog. Only its capture
// owner publishes history; retrieval never advances a second FIFO or clock.
package RGBDKeyframeRetrieval
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RetrieveVisualWords = SLAM.LoopClosure.RetrieveVisualWords;

  constant Integer proposalCapacity = 4;

  function PrepareCapture
    input RGBDKeyframes.Catalog catalog;
    input RGBDKeyframes.Frame measurement;
    input Real vocabulary[RGBDKeyframes.wordCapacity,RGBDKeyframes.descriptorSize];
    input Real vocabularyEnabled[RGBDKeyframes.wordCapacity];
    input Boolean requested;
    input Real maximumWordDistanceSquared = 0.8;
    input Integer minimumAssignments = 8;
    input Real minimumSimilarity = 0.35;
    input Real minimumAge = 5.0;
    output RGBDKeyframes.Frame prepared;
    output Boolean accepted;
    output Integer rejectionReason "0 accepted; 1 not requested; 2 metadata; 3 retrieval/history; 4 frame; 5 candidate binding";
    output Real wordIndex[RGBDKeyframes.featureCapacity];
    output Integer candidateId[proposalCapacity];
    output Integer candidateSlot[proposalCapacity];
    output Real candidateScore[proposalCapacity];
    output Integer candidateCount;
    output Real assignmentCount;
  protected
    Real featureMask[RGBDKeyframes.featureCapacity];
    Real historyMask[RGBDKeyframes.keyframeCapacity];
    Real histogram[RGBDKeyframes.wordCapacity];
    Real ids[proposalCapacity]; Real slots[proposalCapacity]; Real scores[proposalCapacity];
    Real unusedHistogram[RGBDKeyframes.keyframeCapacity,RGBDKeyframes.wordCapacity];
    Real unusedEnabled[RGBDKeyframes.keyframeCapacity];
    Real unusedIds[RGBDKeyframes.keyframeCapacity]; Real unusedTimes[RGBDKeyframes.keyframeCapacity];
    Real unusedVersion; Real unusedSlot; Real unusedTime; Real unusedStored;
    Real configurationValid; Real vocabularyCount; Real retrievalValid; Real count; Real invalidHistoryCount;
    Boolean valid; Integer slot;
  algorithm
    prepared := measurement; accepted := false; rejectionReason := 1;
    wordIndex := zeros(RGBDKeyframes.featureCapacity);
    candidateId := fill(0,proposalCapacity); candidateSlot := fill(0,proposalCapacity);
    candidateScore := zeros(proposalCapacity); candidateCount := 0; assignmentCount := 0;
    if requested then
      rejectionReason := 2;
      valid := RGBDKeyframes.ValidHeader(catalog)
        and catalog.nextId < RGBDKeyframes.identifierLimit
        and measurement.generation == catalog.generation
        and measurement.vocabularyVersion == catalog.vocabularyVersion
        and measurement.id == catalog.nextId and measurement.epoch > catalog.lastEpoch
        and (catalog.nextId == 1 or measurement.imageTime > catalog.lastTime);
      if valid then
        for feature in 1:RGBDKeyframes.featureCapacity loop
          featureMask[feature] := if measurement.enabled[feature] then 1.0 else 0.0;
        end for;
        for node in 1:RGBDKeyframes.keyframeCapacity loop
          // The capture will evict nextSlot. It cannot be a retained loop node
          // in the resulting transaction, so exclude it before query scoring.
          historyMask[node] := if catalog.occupied[node] and node <> catalog.nextSlot then 1.0 else 0.0;
        end for;
        (wordIndex,histogram,ids,slots,scores,unusedHistogram,unusedEnabled,unusedIds,unusedTimes,
          unusedVersion,unusedSlot,unusedTime,configurationValid,vocabularyCount,assignmentCount,
          retrievalValid,count,unusedStored,invalidHistoryCount) := RetrieveVisualWords(
            measurement.descriptor,featureMask,measurement.count,vocabulary,vocabularyEnabled,catalog.vocabularyVersion,
            catalog.histograms,historyMask,catalog.ids,catalog.imageTimes,catalog.vocabularyVersion,
            catalog.nextSlot,catalog.lastTime,measurement.id,measurement.imageTime,0.0,0.0,
            maximumWordDistanceSquared,minimumAssignments,minimumSimilarity,minimumAge);
        rejectionReason := 3;
        valid := configurationValid == 1.0 and retrievalValid == 1.0 and invalidHistoryCount == 0.0
          and count >= 0 and count <= proposalCapacity and floor(count) == count;
        if valid then
          prepared.histogram := histogram;
          rejectionReason := 4;
          valid := RGBDKeyframes.ValidFrame(prepared);
          if valid then
            rejectionReason := 5;
            for rank in 1:proposalCapacity loop
              if rank <= count then
                // Certify Real output domains before Integer conversion or indexing.
                valid := valid and slots[rank] >= 1 and slots[rank] <= RGBDKeyframes.keyframeCapacity
                  and floor(slots[rank]) == slots[rank] and ids[rank] >= 1
                  and ids[rank] < catalog.nextId and floor(ids[rank]) == ids[rank]
                  and scores[rank] >= minimumSimilarity and scores[rank] <= 1.0;
                if valid then
                  slot := integer(slots[rank]);
                  valid := catalog.occupied[slot] and slot <> catalog.nextSlot
                    and catalog.ids[slot] == ids[rank]
                    and catalog.generations[slot] == measurement.generation
                    and catalog.vocabularyVersions[slot] == measurement.vocabularyVersion
                    and measurement.imageTime-catalog.imageTimes[slot] >= minimumAge;
                  if valid then
                    candidateId[rank] := integer(ids[rank]); candidateSlot[rank] := slot;
                    candidateScore[rank] := scores[rank];
                  end if;
                end if;
              end if;
            end for;
            if valid then
              candidateCount := integer(count); accepted := true; rejectionReason := 0;
            end if;
          end if;
        end if;
      end if;
    end if;
    if not accepted then
      prepared := measurement; wordIndex := zeros(RGBDKeyframes.featureCapacity);
      candidateId := fill(0,proposalCapacity); candidateSlot := fill(0,proposalCapacity);
      candidateScore := zeros(proposalCapacity); candidateCount := 0; assignmentCount := 0;
    end if;
  end PrepareCapture;
end RGBDKeyframeRetrieval;
