within Tests.RGBDRetentionTests;
model RGBDRetentionBagOfWordsLockstep "Visual-word history follows the catalog's chosen slot instead of its own ring"
  function run "Store 70 keyframes and their histograms side by side"
    input Integer generation;
    output Boolean passed;
  protected
    import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
    import SLAM.LoopClosure.RetrieveVisualWords;
    constant Integer features = 350; constant Integer size = 49; constant Integer words = 256;
    RGBDKeyframes.Catalog catalog; Boolean accepted; Integer slot; Integer evicted; Integer reason;
    Real descriptor[features,size]; Real descriptorEnabled[features]; Real vocabulary[words,size]; Real vocabularyEnabled[words];
    Real histories[RGBDKeyframes.keyframeCapacity,words]; Real enabled[RGBDKeyframes.keyframeCapacity];
    Real keyframeIds[RGBDKeyframes.keyframeCapacity]; Real keyframeTimes[RGBDKeyframes.keyframeCapacity];
    Real version; Real ring; Real historyTime;
    Real wordIndex[features]; Real histogram[words];
    Real candidateId[4]; Real candidateSlot[4]; Real candidateScore[4];
    Real ringHistories[RGBDKeyframes.keyframeCapacity,words]; Real ringEnabled[RGBDKeyframes.keyframeCapacity];
    Real ringIds[RGBDKeyframes.keyframeCapacity]; Real ringTimes[RGBDKeyframes.keyframeCapacity];
    Real ringVersion; Real ringNext; Real ringTime;
    Real configurationValid; Real vocabularyCount; Real assignmentCount; Real retrievalValid;
    Real candidateCount; Real stored; Real invalidHistoryCount;
  algorithm
    descriptor := zeros(features,size); descriptor[1,1] := 1.0;
    descriptorEnabled := zeros(features); descriptorEnabled[1] := 1.0;
    vocabulary := zeros(words,size); vocabulary[1,1] := 1.0;
    vocabularyEnabled := zeros(words); vocabularyEnabled[1] := 1.0;
    histories := zeros(RGBDKeyframes.keyframeCapacity,words); enabled := zeros(RGBDKeyframes.keyframeCapacity);
    keyframeIds := zeros(RGBDKeyframes.keyframeCapacity); keyframeTimes := zeros(RGBDKeyframes.keyframeCapacity);
    version := 1.0; ring := 1.0; historyTime := 0.0;
    catalog := RGBDKeyframes.Empty(generation);
    for id in 1:70 loop
      (catalog,accepted,slot,evicted,reason) := RGBDKeyframes.Store(catalog,syntheticFrame(id,{0.5*(id-1),0.0,0.0},generation),true);
      assert(accepted,"Catalog Store must succeed");
      (wordIndex,histogram,candidateId,candidateSlot,candidateScore,histories,enabled,keyframeIds,keyframeTimes,
        version,ring,historyTime,configurationValid,vocabularyCount,assignmentCount,retrievalValid,candidateCount,stored,invalidHistoryCount) :=
        RetrieveVisualWords(descriptor,descriptorEnabled,1.0,vocabulary,vocabularyEnabled,1.0,
          histories,enabled,keyframeIds,keyframeTimes,version,ring,historyTime,id,0.3*id,1.0,0.0,1.0,1.0,0.0,0.0,slot);
      assert(configurationValid == 1.0 and stored == 1.0 and invalidHistoryCount == 0.0,"Histogram must be stored");
      for node in 1:RGBDKeyframes.keyframeCapacity loop
        if catalog.occupied[node] then
          assert(enabled[node] == 1.0 and keyframeIds[node] == catalog.ids[node],"Histogram slots must match catalog slots");
        else
          assert(enabled[node] == 0.0,"Free catalog slots have no histogram");
        end if;
      end for;
    end for;
    // The standalone ring would overwrite its own next slot, not the slot the
    // catalog chose for id 71.
    (wordIndex,histogram,candidateId,candidateSlot,candidateScore,ringHistories,ringEnabled,ringIds,ringTimes,
      ringVersion,ringNext,ringTime,configurationValid,vocabularyCount,assignmentCount,retrievalValid,candidateCount,stored,invalidHistoryCount) :=
      RetrieveVisualWords(descriptor,descriptorEnabled,1.0,vocabulary,vocabularyEnabled,1.0,
        histories,enabled,keyframeIds,keyframeTimes,version,ring,historyTime,71,0.3*71,1.0,0.0,1.0,1.0,0.0,0.0);
    assert(stored == 1.0 and integer(ring) <> catalog.nextSlot,"The ring and catalog choices must differ in this run");
    assert(ringIds[integer(ring)] == 71 and ringIds[catalog.nextSlot] == catalog.ids[catalog.nextSlot],
      "Without catalogSlot the ring slot is overwritten");
    (wordIndex,histogram,candidateId,candidateSlot,candidateScore,ringHistories,ringEnabled,ringIds,ringTimes,
      ringVersion,ringNext,ringTime,configurationValid,vocabularyCount,assignmentCount,retrievalValid,candidateCount,stored,invalidHistoryCount) :=
      RetrieveVisualWords(descriptor,descriptorEnabled,1.0,vocabulary,vocabularyEnabled,1.0,
        histories,enabled,keyframeIds,keyframeTimes,version,ring,historyTime,71,0.3*71,1.0,0.0,1.0,1.0,0.0,0.0,65.0);
    assert(configurationValid == 0.0,"A catalog slot outside 1..64 is a configuration error");
    passed := true;
  end run;
  // Evaluated once at initialization; Evaluate=false keeps the catalog
  // workload out of translation-time constant folding.
  parameter Integer generation = 1 "Catalog generation" annotation(Evaluate=false);
  parameter Boolean passed(fixed=false);
initial equation
  passed = run(generation);
equation
  assert(passed,"Bag-of-words lockstep checks did not complete");
end RGBDRetentionBagOfWordsLockstep;
