within SLAM.Mapping;
function UpdateLandmarkMapWithReceipts
  import RGBDLandmarkMapInterface = SLAM.Mapping.RGBDLandmarkMapInterface;
  import RGBDSpatialIndex = SLAM.Mapping.RGBDSpatialIndex;

  extends RGBDLandmarkMapInterface;
  output Integer insertedFeature[size(previousOccupied,1)] "Original candidate slot, zero for retained/empty slots";
protected
  Boolean configurationValid;
  Boolean stateValid;
  Boolean slotValid;
  Boolean reset;
  Boolean initialObservation;
  Boolean present;
  Boolean keep;
  Boolean candidateValid;
  Boolean insert;
  Boolean merge;
  Real safeCandidate[3];
  Real difference[3];
  Real distanceSquared;
  Real lifetime;
  Integer spatialHead[RGBDSpatialIndex.defaultBucketCount];
  Integer spatialLink[size(previousOccupied,1)];
  Integer spatialCell[size(previousOccupied,1),3];
  Integer freeNext[size(previousOccupied,1)];
  Integer firstFree;
  Integer visited;
  Integer bucket;
  Boolean indexValid;
  Boolean queryValid;
  Boolean cellValid;
  Boolean linearSearch;
  Integer duplicate;
  Integer vacant;
  Integer destination;
algorithm
  reset := resetRequested >= 1.0 and resetRequested <= 1.0;
  // Only the first observation may share the empty map's zero timestamp.
  // State validation below rules out every occupied slot at previousFrame=0.
  // Later observations still require strictly increasing acquisition time.
  initialObservation := previousFrame == 0.0 and previousTime == 0.0 and timeNow == 0.0;
  slotValid := false; present := false; keep := false;
  candidateValid := false; insert := false; merge := false;
  safeCandidate := zeros(3); difference := zeros(3);
  distanceSquared := 0.0; lifetime := 0.0;
  duplicate := 0; vacant := 0; destination := 1;
  spatialHead := fill(0,RGBDSpatialIndex.defaultBucketCount);
  spatialLink := fill(0,size(previousOccupied,1));
  spatialCell := fill(0,size(previousOccupied,1),3);
  freeNext := fill(0,size(previousOccupied,1));
  firstFree := 0; visited := 0; bucket := 1;
  indexValid := true; queryValid := false; cellValid := false; linearSearch := false;
  confirmed := zeros(size(previousOccupied,1));
  insertedFeature := fill(0,size(previousOccupied,1));
  configurationValid := size(previousPoint,1) == size(previousOccupied,1) and size(previousPoint,2) == 3
    and size(previousConfidence,1) == size(previousOccupied,1) and size(previousLastSeen,1) == size(previousOccupied,1)
    and size(previousLastFrame,1) == size(previousOccupied,1) and size(candidatePoint,1) == size(candidateEnabled,1)
    and size(candidatePoint,2) == 3 and size(previousOccupied,1) > 0
    and size(previousOccupied,1) <= RGBDSpatialIndex.maximumSlotCount
    and coordinateLimit > 0.0 and coordinateLimit <= 1e6 and voxelWidth >= 0.001 and voxelWidth <= 10.0
    and mergeRadius >= 0.0 and mergeRadius <= 10.0 and maximumDistance > 0.0 and maximumDistance <= 1e4
    and tentativeLifetime > 0.0 and tentativeLifetime <= 1e4 and confirmedLifetime >= tentativeLifetime and confirmedLifetime <= 1e4
    and confirmationObservations >= 2.0 and confirmationObservations <= maximumConfidence and floor(confirmationObservations) == confirmationObservations
    and maximumConfidence <= 100.0 and floor(maximumConfidence) == maximumConfidence
    and maximumTentative >= 0.0 and maximumTentative <= size(previousOccupied,1) and floor(maximumTentative) == maximumTentative
    and candidateCount >= 0.0 and candidateCount <= size(candidateEnabled,1) and floor(candidateCount) == candidateCount
    and poseAccepted >= 1.0 and poseAccepted <= 1.0
    and abs(bodyPosition[1]) <= coordinateLimit and abs(bodyPosition[2]) <= coordinateLimit and abs(bodyPosition[3]) <= coordinateLimit
    and timeNow >= 0.0 and timeNow <= 1e9
    and worldFrame >= 0.0 and worldFrame <= 1e9 and floor(worldFrame) == worldFrame
    and ((reset and frameNow == 1.0) or (resetRequested >= 0.0 and resetRequested <= 0.0
      and previousTime >= 0.0 and previousTime <= 1e9 and (previousTime < timeNow or initialObservation)
      and previousFrame >= 0.0 and previousFrame < 1e9 and floor(previousFrame) == previousFrame and frameNow == previousFrame+1.0
      and previousWorldFrame == worldFrame));
  stateValid := true;
  for slot in 1:size(previousOccupied,1) loop
    slotValid := previousOccupied[slot] >= 0.0 and previousOccupied[slot] <= 0.0
      or (previousOccupied[slot] >= 1.0 and previousOccupied[slot] <= 1.0
        and abs(previousPoint[slot,1]) <= coordinateLimit and abs(previousPoint[slot,2]) <= coordinateLimit and abs(previousPoint[slot,3]) <= coordinateLimit
        and previousConfidence[slot] >= 1.0 and previousConfidence[slot] <= maximumConfidence and floor(previousConfidence[slot]) == previousConfidence[slot]
        and previousLastSeen[slot] >= 0.0 and previousLastSeen[slot] <= previousTime
        and previousLastFrame[slot] >= 1.0 and previousLastFrame[slot] <= previousFrame and floor(previousLastFrame[slot]) == previousLastFrame[slot]);
    stateValid := stateValid and (reset or slotValid);
  end for;
  accepted := if configurationValid and stateValid then 1.0 else 0.0;
  rejectionReason := if not configurationValid then 1.0 else if not stateValid then 2.0 else 0.0;
  point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
  lastSeen := previousLastSeen; lastFrame := previousLastFrame;
  nextTime := if accepted > 0.5 then timeNow else previousTime;
  nextFrame := if accepted > 0.5 then frameNow else previousFrame;
  nextWorldFrame := if accepted > 0.5 then worldFrame else previousWorldFrame;
  insertedCount := 0.0; mergedCount := 0.0; prunedCount := 0.0; droppedCount := 0.0; invalidCandidateCount := 0.0;
  tentativeCount := 0.0;
  // Prune before insertion. Empty storage is canonicalized only on acceptance.
  for slot in 1:size(previousOccupied,1) loop
    present := accepted > 0.5 and not reset and previousOccupied[slot] > 0.5;
    difference := if present then previousPoint[slot,:]-bodyPosition else zeros(3);
    distanceSquared := difference[1]*difference[1]+difference[2]*difference[2]+difference[3]*difference[3];
    lifetime := if present and previousConfidence[slot] >= confirmationObservations then confirmedLifetime else tentativeLifetime;
    keep := present and timeNow-previousLastSeen[slot] <= lifetime and distanceSquared <= maximumDistance*maximumDistance;
    prunedCount := prunedCount+(if present and not keep then 1.0 else 0.0);
    point[slot,:] := if accepted > 0.5 and not keep then zeros(3) else point[slot,:];
    occupied[slot] := if accepted > 0.5 then (if keep then 1.0 else 0.0) else occupied[slot];
    confidence[slot] := if accepted > 0.5 and not keep then 0.0 else confidence[slot];
    lastSeen[slot] := if accepted > 0.5 and not keep then 0.0 else lastSeen[slot];
    lastFrame[slot] := if accepted > 0.5 and not keep then 0.0 else lastFrame[slot];
    tentativeCount := tentativeCount+(if keep and confidence[slot] < confirmationObservations then 1.0 else 0.0);
  end for;
  // Build once after pruning. The index is local to this transaction, so it
  // cannot become stale across a reset, accepted pose or retained map update.
  if accepted > 0.5 then
    (spatialHead,spatialLink,spatialCell,freeNext,firstFree,indexValid) :=
      RGBDSpatialIndex.Build(point,occupied,voxelWidth);
  end if;
  // Raster candidate order is stable, including sparse final slots. Repeated
  // candidates in one frame never count as independent confirmation evidence.
  for feature in 1:size(candidateEnabled,1) loop
    candidateValid := accepted > 0.5 and indexValid and feature <= candidateCount and candidateEnabled[feature] >= 1.0 and candidateEnabled[feature] <= 1.0
      and abs(candidatePoint[feature,1]) <= coordinateLimit and abs(candidatePoint[feature,2]) <= coordinateLimit and abs(candidatePoint[feature,3]) <= coordinateLimit;
    invalidCandidateCount := invalidCandidateCount+(if accepted > 0.5 and feature <= candidateCount
      and not (candidateEnabled[feature] >= 0.0 and candidateEnabled[feature] <= 0.0) and not candidateValid then 1.0 else 0.0);
    safeCandidate := if candidateValid then candidatePoint[feature,:] else zeros(3);
    difference := if candidateValid then safeCandidate-bodyPosition else zeros(3);
    distanceSquared := difference[1]*difference[1]+difference[2]*difference[2]+difference[3]*difference[3];
    candidateValid := candidateValid and distanceSquared <= maximumDistance*maximumDistance;
    duplicate := 0; vacant := 0;
    if candidateValid then
      (duplicate,queryValid,visited,linearSearch) := RGBDSpatialIndex.Find(
        point,occupied,spatialHead,spatialLink,spatialCell,safeCandidate,voxelWidth,mergeRadius,true);
      indexValid := indexValid and queryValid;
      candidateValid := candidateValid and queryValid;
      vacant := if queryValid then firstFree else 0;
    end if;
    merge := candidateValid and duplicate > 0;
    insert := candidateValid and duplicate == 0 and vacant > 0 and tentativeCount < maximumTentative;
    destination := if duplicate > 0 then duplicate else if vacant > 0 then vacant else 1;
    tentativeCount := tentativeCount+(if insert then 1.0 else if merge and lastFrame[destination] < frameNow
      and confidence[destination] < confirmationObservations and confidence[destination]+1.0 >= confirmationObservations then -1.0 else 0.0);
    point[destination,:] := if insert then safeCandidate else point[destination,:];
    occupied[destination] := if insert then 1.0 else occupied[destination];
    confidence[destination] := if insert then 1.0 else if merge and lastFrame[destination] < frameNow
      then min(maximumConfidence,confidence[destination]+1.0) else confidence[destination];
    lastSeen[destination] := if insert or merge then timeNow else lastSeen[destination];
    lastFrame[destination] := if insert or merge then frameNow else lastFrame[destination];
    insertedFeature[destination] := if insert then feature else insertedFeature[destination];
    insertedCount := insertedCount+(if insert then 1.0 else 0.0);
    mergedCount := mergedCount+(if merge then 1.0 else 0.0);
    droppedCount := droppedCount+(if candidateValid and not insert and not merge then 1.0 else 0.0);
    if insert then
      // A merge keeps the original anchor; only insertions change the index.
      // Update local arrays instead of returning/copying a table per feature.
      (spatialCell[destination,:],cellValid) := RGBDSpatialIndex.Cell(safeCandidate,voxelWidth);
      indexValid := indexValid and cellValid;
      if cellValid then
        bucket := RGBDSpatialIndex.Bucket(spatialCell[destination,:],size(spatialHead,1));
        spatialLink[destination] := spatialHead[bucket];
        spatialHead[bucket] := destination;
        firstFree := freeNext[destination];
        freeNext[destination] := 0;
      end if;
    end if;
  end for;
  // Refuse an invalid private index atomically, just like invalid public state.
  if accepted > 0.5 and not indexValid then
    accepted := 0.0; rejectionReason := 3.0;
    point := previousPoint; occupied := previousOccupied; confidence := previousConfidence;
    lastSeen := previousLastSeen; lastFrame := previousLastFrame;
    nextTime := previousTime; nextFrame := previousFrame; nextWorldFrame := previousWorldFrame;
    insertedCount := 0.0; mergedCount := 0.0; prunedCount := 0.0;
    droppedCount := 0.0; invalidCandidateCount := 0.0;
    insertedFeature := fill(0,size(previousOccupied,1));
  end if;
  occupiedCount := 0.0; confirmedCount := 0.0; tentativeCount := 0.0;
  for slot in 1:size(previousOccupied,1) loop
    confirmed[slot] := if occupied[slot] >= 1.0 and occupied[slot] <= 1.0 and confidence[slot] >= confirmationObservations then 1.0 else 0.0;
    occupiedCount := occupiedCount+(if occupied[slot] >= 1.0 and occupied[slot] <= 1.0 then 1.0 else 0.0);
    confirmedCount := confirmedCount+confirmed[slot];
    tentativeCount := tentativeCount+(if occupied[slot] >= 1.0 and occupied[slot] <= 1.0 and confirmed[slot] < 0.5 then 1.0 else 0.0);
  end for;
end UpdateLandmarkMapWithReceipts;
