within SLAM.LoopClosure;
// Measured appearance bootstrap. Word identities freeze before histogram storage.
// This is not a loop constraint, geometric observation or scene/truth signature.
package RGBDVisualVocabulary
  import NormalizeVisualWord = SLAM.LoopClosure.NormalizeVisualWord;
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;

  constant Integer vocabularyCapacity = RGBDKeyframes.wordCapacity;
  constant Integer descriptorSize = RGBDKeyframes.descriptorSize;
  constant Integer featureCapacity = RGBDKeyframes.featureCapacity;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record State
    Integer generation;
    Integer sourceRevision;
    Integer version;
    Real words[vocabularyCapacity,descriptorSize];
    Real enabled[vocabularyCapacity];
    Integer count;
    Boolean ready "Once true, this dictionary cannot be changed by Learn";
  end State;

  function Empty
    input Integer generation = 1;
    input Integer sourceRevision = 1;
    input Integer version = 1;
    output State state;
  algorithm
    // Invalid supplied ownership remains invalid: Valid/Learn refuse it.
    state.generation := generation; state.sourceRevision := sourceRevision;
    state.version := version; state.words := zeros(vocabularyCapacity,descriptorSize);
    state.enabled := zeros(vocabularyCapacity); state.count := 0; state.ready := false;
  end Empty;

  function Valid
    input State state;
    output Boolean valid;
  protected
    Real mean; Real energy;
  algorithm
    valid := state.generation >= 1 and state.generation <= identifierLimit
      and state.sourceRevision >= 1 and state.sourceRevision <= identifierLimit
      and state.version >= 1 and state.version <= identifierLimit
      and state.count >= 0 and state.count <= vocabularyCapacity
      and (not state.ready or state.count >= 1);
    mean := 0.0; energy := 0.0;
    for word in 1:vocabularyCapacity loop
      valid := valid and state.enabled[word] == (if word <= state.count then 1.0 else 0.0);
      // Disabled word storage is opaque, including restored padding.
      if word <= state.count then
        mean := 0.0; energy := 0.0;
        for component in 1:descriptorSize loop
          valid := valid and abs(state.words[word,component]) <= 1.0;
          mean := mean+state.words[word,component]/descriptorSize;
          energy := energy+state.words[word,component]*state.words[word,component];
        end for;
        valid := valid and abs(mean) <= 1e-6 and abs(energy-1.0) <= 1e-6;
      end if;
    end for;
  end Valid;

  function Learn
    input State previous;
    input Real descriptor[featureCapacity,descriptorSize];
    input Real descriptorEnabled[featureCapacity];
    input Real descriptorCount "Full domain extent, including disabled sparse slots";
    input Integer generation;
    input Integer sourceRevision;
    input Integer version;
    input Boolean requested = true;
    input Integer minimumMeasuredDescriptors = 8;
    input Real minimumDistanceSquared = 0.04;
    output State next;
    output Boolean accepted;
    output Integer reason "0 idle, 1 frozen now, 2 learning, 3 already frozen; negative refusal";
  protected
    State pending;
    Real samples[featureCapacity,descriptorSize];
    Real sample[descriptorSize]; Boolean sampleValid; Boolean inputsValid;
    Integer domainCount; Integer measuredCount;
    Real distance; Real nearest;
  algorithm
    next := previous; accepted := false; reason := 0;
    pending := previous; samples := zeros(featureCapacity,descriptorSize);
    sample := zeros(descriptorSize); sampleValid := false; inputsValid := true;
    domainCount := 0; measuredCount := 0; distance := 0.0; nearest := 5.0;
    if requested then
      if not Valid(previous) then
        reason := -1;
      elseif generation < 1 or generation > identifierLimit or sourceRevision < 1
        or sourceRevision > identifierLimit or version < 1 or version > identifierLimit
        or generation <> previous.generation or sourceRevision <> previous.sourceRevision
        or version <> previous.version then
        reason := -2;
      elseif minimumMeasuredDescriptors < 1 or minimumMeasuredDescriptors > featureCapacity
        or not (minimumDistanceSquared >= 1e-12 and minimumDistanceSquared <= 4.0) then
        reason := -3;
      elseif not (descriptorCount >= 0.0 and descriptorCount <= featureCapacity
        and floor(descriptorCount) == descriptorCount) then
        reason := -4;
      else
        domainCount := integer(descriptorCount);
        // Complete preflight before adding even the first word. No enabled slot
        // outside the supplied domain, fractional mask or active poison can commit.
        for feature in 1:featureCapacity loop
          inputsValid := inputsValid and (descriptorEnabled[feature] == 0.0
            or descriptorEnabled[feature] == 1.0)
            and (feature <= domainCount or descriptorEnabled[feature] == 0.0);
          if feature <= domainCount and descriptorEnabled[feature] == 1.0 then
            (sample,sampleValid) := NormalizeVisualWord(descriptor[feature,:],1.0);
            inputsValid := inputsValid and sampleValid;
            samples[feature,:] := sample;
            measuredCount := measuredCount+1;
          end if;
        end for;
        if not inputsValid then
          reason := -4;
        elseif previous.ready then
          accepted := true; reason := 3;
        else
          // Fixed-bounded nearest representative admission, source raster order.
          // Identical samples count as measurements but do not consume more words.
          for feature in 1:featureCapacity loop
            if feature <= domainCount and descriptorEnabled[feature] == 1.0
              and pending.count < vocabularyCapacity then
              nearest := 5.0;
              for word in 1:vocabularyCapacity loop
                if word <= pending.count then
                  distance := 0.0;
                  for component in 1:descriptorSize loop
                    distance := distance+(samples[feature,component]-pending.words[word,component])
                      *(samples[feature,component]-pending.words[word,component]);
                  end for;
                  nearest := min(nearest,distance);
                end if;
              end for;
              if pending.count == 0 or nearest >= minimumDistanceSquared then
                pending.count := pending.count+1;
                pending.words[pending.count,:] := samples[feature,:];
                pending.enabled[pending.count] := 1.0;
              end if;
            end if;
          end for;
          pending.ready := measuredCount >= minimumMeasuredDescriptors and pending.count >= 1;
          next := pending; accepted := true; reason := if pending.ready then 1 else 2;
        end if;
      end if;
    end if;
  end Learn;

  function Bootstrap
    input State previous;
    input Real descriptor[featureCapacity,descriptorSize];
    input Real descriptorEnabled[featureCapacity]; input Real descriptorCount;
    input Integer generation; input Integer sourceRevision; input Integer version;
    input Boolean requested = true;
    input Integer minimumMeasuredDescriptors = 8; input Real minimumDistanceSquared = 0.04;
    output State next; output Boolean accepted; output Integer reason;
  algorithm
    (next,accepted,reason) := Learn(previous,descriptor,descriptorEnabled,descriptorCount,
      generation,sourceRevision,version,requested,minimumMeasuredDescriptors,minimumDistanceSquared);
  end Bootstrap;
end RGBDVisualVocabulary;
