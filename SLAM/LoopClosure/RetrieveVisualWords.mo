within SLAM.LoopClosure;
function RetrieveVisualWords
  import NormalizeVisualWord = SLAM.LoopClosure.NormalizeVisualWord;

  input Real descriptor[featureCapacity,descriptorSize];
  input Real descriptorEnabled[featureCapacity];
  input Real descriptorCount;
  input Real vocabulary[vocabularyCapacity,descriptorSize];
  input Real vocabularyEnabled[vocabularyCapacity];
  input Real vocabularyVersion;
  input Real previousHistogram[keyframeCapacity,vocabularyCapacity];
  input Real previousEnabled[keyframeCapacity];
  input Real previousKeyframeId[keyframeCapacity];
  input Real previousKeyframeTime[keyframeCapacity];
  input Real previousVersion;
  input Real previousNextSlot;
  input Real previousTime;
  input Real queryKeyframeId;
  input Real timeNow;
  input Real storeKeyframe;
  input Real resetRequested;
  input Real maximumWordDistanceSquared;
  input Real minimumAssignments;
  input Real minimumSimilarity;
  input Real minimumAge;
  output Real wordIndex[featureCapacity];
  output Real histogram[vocabularyCapacity];
  output Real candidateId[candidateCapacity];
  output Real candidateSlot[candidateCapacity];
  output Real candidateScore[candidateCapacity];
  output Real nextHistogram[keyframeCapacity,vocabularyCapacity];
  output Real nextEnabled[keyframeCapacity];
  output Real nextKeyframeId[keyframeCapacity];
  output Real nextKeyframeTime[keyframeCapacity];
  output Real nextVersion;
  output Real nextSlot;
  output Real nextTime;
  output Real configurationValid;
  output Real vocabularyCount;
  output Real assignmentCount;
  output Real retrievalValid;
  output Real candidateCount;
  output Real stored;
  output Real invalidHistoryCount;
protected
  constant Integer featureCapacity = 350;
  constant Integer descriptorSize = 49;
  constant Integer vocabularyCapacity = 256;
  constant Integer keyframeCapacity = 128;
  constant Integer candidateCapacity = 4;
  Real words[vocabularyCapacity,descriptorSize];
  Real usableWord[vocabularyCapacity];
  Real sample[descriptorSize];
  Boolean sampleValid;
  Real distance;
  Real nearestDistanceSquared;
  Integer nearestWord;
  Real histogramMass;
  Real idf[vocabularyCapacity];
  Real documentFrequency;
  Real documentCount;
  Real queryNormSquared;
  Real weightedDot;
  Real candidateNormSquared;
  Real weighted;
  Real score[keyframeCapacity];
  Real selected[keyframeCapacity];
  Real bestScore;
  Real bestId;
  Integer bestSlot;
  Integer destination;
  Integer existingSlot;
  Boolean configuration;
  Boolean stateValid;
  Boolean resetHistory;
  Boolean historyValid;
  Boolean eligible;
algorithm
  wordIndex := zeros(featureCapacity);
  histogram := zeros(vocabularyCapacity);
  candidateId := zeros(candidateCapacity);
  candidateSlot := zeros(candidateCapacity);
  candidateScore := zeros(candidateCapacity);
  words := zeros(vocabularyCapacity,descriptorSize);
  usableWord := zeros(vocabularyCapacity);
  sample := zeros(descriptorSize);
  sampleValid := false;
  nextHistogram := zeros(keyframeCapacity,vocabularyCapacity);
  nextEnabled := zeros(keyframeCapacity);
  nextKeyframeId := zeros(keyframeCapacity);
  nextKeyframeTime := zeros(keyframeCapacity);
  idf := zeros(vocabularyCapacity);
  score := zeros(keyframeCapacity);
  selected := zeros(keyframeCapacity);
  nextVersion := 0.0;
  nextSlot := 1.0;
  nextTime := 0.0;
  configurationValid := 0.0;
  vocabularyCount := 0.0;
  assignmentCount := 0.0;
  retrievalValid := 0.0;
  candidateCount := 0.0;
  stored := 0.0;
  invalidHistoryCount := 0.0;
  distance := 0.0;
  nearestDistanceSquared := 5.0;
  nearestWord := 0;
  histogramMass := 0.0;
  documentFrequency := 0.0;
  documentCount := 0.0;
  queryNormSquared := 0.0;
  weightedDot := 0.0;
  candidateNormSquared := 0.0;
  weighted := 0.0;
  bestScore := -1.0;
  bestId := 1e9+1.0;
  bestSlot := 0;
  destination := 1;
  existingSlot := 0;
  historyValid := false;
  eligible := false;
  stateValid := previousVersion >= 0.0 and previousVersion <= 1e9 and floor(previousVersion) == previousVersion
    and previousNextSlot >= 1.0 and previousNextSlot <= keyframeCapacity and floor(previousNextSlot) == previousNextSlot
    and previousTime >= 0.0 and previousTime <= 1e9;
  configuration := descriptorCount >= 0.0 and descriptorCount <= featureCapacity and floor(descriptorCount) == descriptorCount
    and vocabularyVersion >= 1.0 and vocabularyVersion <= 1e9 and floor(vocabularyVersion) == vocabularyVersion
    and queryKeyframeId >= 0.0 and queryKeyframeId <= 1e9 and floor(queryKeyframeId) == queryKeyframeId
    and timeNow >= 0.0 and timeNow <= 1e9 and (timeNow >= previousTime or resetRequested == 1.0)
    and (storeKeyframe == 0.0 or storeKeyframe == 1.0) and (storeKeyframe == 0.0 or queryKeyframeId >= 1.0)
    and (resetRequested == 0.0 or resetRequested == 1.0) and (stateValid or resetRequested == 1.0)
    and maximumWordDistanceSquared >= 0.0 and maximumWordDistanceSquared <= 4.0
    and minimumAssignments >= 1.0 and minimumAssignments <= featureCapacity and floor(minimumAssignments) == minimumAssignments
    and minimumSimilarity >= 0.0 and minimumSimilarity <= 1.0 and minimumAge >= 0.0 and minimumAge <= 1e6;
  resetHistory := configuration and (resetRequested == 1.0 or vocabularyVersion <> previousVersion);
  nextVersion := if configuration then vocabularyVersion else if stateValid then previousVersion else 0.0;
  nextSlot := if stateValid and not resetHistory then previousNextSlot else 1.0;
  nextTime := if configuration then timeNow else if stateValid then previousTime else 0.0;
  // Validate retained histograms before retrieval.
  for frame in 1:keyframeCapacity loop
    histogramMass := 0.0;
    historyValid := stateValid and not resetHistory and previousEnabled[frame] == 1.0
      and previousKeyframeId[frame] >= 1.0 and previousKeyframeId[frame] <= 1e9
      and floor(previousKeyframeId[frame]) == previousKeyframeId[frame]
      and previousKeyframeTime[frame] >= 0.0 and previousKeyframeTime[frame] <= previousTime;
    for word in 1:vocabularyCapacity loop
      historyValid := historyValid and previousHistogram[frame,word] >= 0.0 and previousHistogram[frame,word] <= 1.0;
      histogramMass := histogramMass+(if previousHistogram[frame,word] >= 0.0 and previousHistogram[frame,word] <= 1.0
        then previousHistogram[frame,word] else 0.0);
    end for;
    historyValid := historyValid and abs(histogramMass-1.0) <= 1e-6;
    invalidHistoryCount := invalidHistoryCount+(if not resetHistory and previousEnabled[frame] <> 0.0 and not historyValid then 1.0 else 0.0);
    nextEnabled[frame] := if historyValid then 1.0 else 0.0;
    nextKeyframeId[frame] := if historyValid then previousKeyframeId[frame] else 0.0;
    nextKeyframeTime[frame] := if historyValid then previousKeyframeTime[frame] else 0.0;
    for word in 1:vocabularyCapacity loop
      nextHistogram[frame,word] := if historyValid then previousHistogram[frame,word] else 0.0;
    end for;
    documentCount := documentCount+nextEnabled[frame];
    if nextEnabled[frame] == 1.0 and nextKeyframeId[frame] == queryKeyframeId and existingSlot == 0 then
      existingSlot := frame;
    end if;
  end for;
  for word in 1:vocabularyCapacity loop
    (sample,sampleValid) := NormalizeVisualWord(vocabulary[word,:],if configuration then vocabularyEnabled[word] else 0.0);
    words[word,:] := sample;
    usableWord[word] := if sampleValid then 1.0 else 0.0;
    vocabularyCount := vocabularyCount+usableWord[word];
  end for;
  // Nearest visual word; distance ties keep the lower index.
  for feature in 1:featureCapacity loop
    (sample,sampleValid) := NormalizeVisualWord(descriptor[feature,:],
      if configuration and feature <= descriptorCount then descriptorEnabled[feature] else 0.0);
    nearestDistanceSquared := 5.0;
    nearestWord := 0;
    for word in 1:vocabularyCapacity loop
      if sampleValid and usableWord[word] == 1.0 then
        distance := 0.0;
        for k in 1:descriptorSize loop
          distance := distance+(sample[k]-words[word,k])*(sample[k]-words[word,k]);
        end for;
        if distance < nearestDistanceSquared then
          nearestDistanceSquared := distance;
          nearestWord := word;
        end if;
      end if;
    end for;
    if nearestWord > 0 and nearestDistanceSquared <= maximumWordDistanceSquared then
      wordIndex[feature] := nearestWord;
      histogram[nearestWord] := histogram[nearestWord]+1.0;
      assignmentCount := assignmentCount+1.0;
    end if;
  end for;
  // TF-IDF query weights.
  for word in 1:vocabularyCapacity loop
    histogram[word] := if assignmentCount > 0.0 then histogram[word]/max(assignmentCount,1.0) else 0.0;
    documentFrequency := 0.0;
    for frame in 1:keyframeCapacity loop
      documentFrequency := documentFrequency+(if nextEnabled[frame] == 1.0 and nextHistogram[frame,word] > 0.0 then 1.0 else 0.0);
    end for;
    idf[word] := 1.0+log((1.0+documentCount)/(1.0+documentFrequency));
    weighted := histogram[word]*idf[word];
    queryNormSquared := queryNormSquared+weighted*weighted;
  end for;
  configurationValid := if configuration then 1.0 else 0.0;
  retrievalValid := if configuration and vocabularyCount > 0.0
    and assignmentCount >= minimumAssignments and queryNormSquared > 1e-12 then 1.0 else 0.0;
  // Cosine similarity against eligible retained keyframes.
  for frame in 1:keyframeCapacity loop
    eligible := retrievalValid == 1.0 and nextEnabled[frame] == 1.0
      and timeNow-nextKeyframeTime[frame] >= minimumAge and queryKeyframeId <> nextKeyframeId[frame];
    weightedDot := 0.0;
    candidateNormSquared := 0.0;
    for word in 1:vocabularyCapacity loop
      weighted := nextHistogram[frame,word]*idf[word];
      candidateNormSquared := candidateNormSquared+weighted*weighted;
      weightedDot := weightedDot+weighted*histogram[word]*idf[word];
    end for;
    score[frame] := if eligible and candidateNormSquared > 1e-12
      then min(1.0,max(0.0,weightedDot/sqrt(max(queryNormSquared*candidateNormSquared,1e-24)))) else -1.0;
  end for;
  for rank in 1:candidateCapacity loop
    bestScore := -1.0;
    bestId := 1e9+1.0;
    bestSlot := 0;
    for frame in 1:keyframeCapacity loop
      if selected[frame] == 0.0 and score[frame] >= minimumSimilarity
        and (score[frame] > bestScore or (score[frame] == bestScore and
          (nextKeyframeId[frame] < bestId or (nextKeyframeId[frame] == bestId and frame < bestSlot)))) then
        bestScore := score[frame];
        bestId := nextKeyframeId[frame];
        bestSlot := frame;
      end if;
    end for;
    if bestSlot > 0 then
      candidateId[rank] := bestId;
      candidateSlot[rank] := bestSlot;
      candidateScore[rank] := bestScore;
      selected[bestSlot] := 1.0;
      candidateCount := candidateCount+1.0;
    end if;
  end for;
  // Query history before inserting the geometrically admitted keyframe.
  destination := if existingSlot > 0 then existingSlot else if nextSlot >= 1.0 and nextSlot <= keyframeCapacity then integer(nextSlot) else 1;
  stored := if retrievalValid == 1.0 and storeKeyframe == 1.0 then 1.0 else 0.0;
  if stored == 1.0 then
    nextHistogram[destination,:] := histogram;
    nextEnabled[destination] := 1.0;
    nextKeyframeId[destination] := queryKeyframeId;
    nextKeyframeTime[destination] := timeNow;
    nextSlot := if existingSlot > 0 then nextSlot else if destination < keyframeCapacity then destination+1.0 else 1.0;
  end if;
end RetrieveVisualWords;
