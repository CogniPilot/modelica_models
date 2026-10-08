within SLAM.Mapping;
// Consume private map insertion receipts. Anchor outputs are a proposal until
// the surrounding owner commits map geometry, metadata and anchors together.
function AssignLandmarkAnchors
  import RGBDMapAnchors = SLAM.Mapping.RGBDMapAnchors;

  input Real previousPoint[:,3];
  input Real previousOccupied[size(previousPoint,1)];
  input Real previousLocalPoint[size(previousPoint,1),3];
  input Integer previousAnchorId[size(previousPoint,1)];
  input Integer previousAnchorSlot[size(previousPoint,1)];
  input Integer previousGeneration;
  input Real nextPoint[size(previousPoint,1),3];
  input Real nextOccupied[size(previousPoint,1)];
  input Integer insertedFeature[size(previousPoint,1)];
  input Real candidatePoint[:,3];
  input Real candidateEnabled[size(candidatePoint,1)]; input Real candidateCount;
  input Boolean nodeEnabled[:]; input Integer nodeId[size(nodeEnabled,1)];
  input Real nodePosition[size(nodeEnabled,1),3]; input Real nodeRotation[size(nodeEnabled,1),3,3];
  input Integer catalogGeneration; input Integer generation;
  input Integer selectedAnchorId; input Integer selectedAnchorSlot;
  input Boolean mapAccepted; input Boolean requested; input Boolean reset;
  input Real coordinateLimit; input Real consistencyTolerance;
  output Real localPoint[size(previousPoint,1),3];
  output Integer anchorId[size(previousPoint,1)]; output Integer anchorSlot[size(previousPoint,1)];
  output Integer nextGeneration; output Boolean accepted; output Integer rejectionReason;
  output Integer assignedCount; output Integer retainedCount; output Integer clearedCount;
protected
  constant Integer featureCapacity = 350;
  Boolean configuration; Boolean valid; Boolean slotValid; Boolean selectedValid;
  Boolean seen[size(candidatePoint,1)];
  Integer feature; Integer owner; Real local[3]; Real reconstructed[3];
algorithm
  localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
  nextGeneration := previousGeneration; accepted := false; rejectionReason := 1;
  assignedCount := 0; retainedCount := 0; clearedCount := 0;
  if requested then
    configuration := size(previousPoint,1) >= 1 and size(previousPoint,1) <= RGBDMapAnchors.mapCapacity
      and size(candidatePoint,1) >= 1 and size(candidatePoint,1) <= featureCapacity
      and size(nodeEnabled,1) >= 1 and size(nodeEnabled,1) <= RGBDMapAnchors.keyframeCapacity
      and candidateCount >= 0.0 and candidateCount <= size(candidatePoint,1) and floor(candidateCount) == candidateCount
      and generation >= 1 and generation <= RGBDMapAnchors.identifierLimit and catalogGeneration == generation
      and ((reset and previousGeneration >= 0 and previousGeneration < RGBDMapAnchors.identifierLimit
        and generation == previousGeneration+1) or (not reset and previousGeneration == generation))
      and selectedAnchorSlot >= 0 and selectedAnchorSlot <= size(nodeEnabled,1)
      and selectedAnchorId >= 0 and selectedAnchorId <= RGBDMapAnchors.identifierLimit
      and coordinateLimit > 0.0 and coordinateLimit <= 1e6
      and consistencyTolerance > 0.0 and consistencyTolerance <= 0.01;
    rejectionReason := 2;
    if configuration then
      rejectionReason := 3;
      if mapAccepted then
        rejectionReason := 4;
        if RGBDMapAnchors.ValidNodes(nodeEnabled,nodeId,nodePosition,nodeRotation,coordinateLimit) then
          selectedValid := false;
          if selectedAnchorSlot >= 1 and selectedAnchorId >= 1 then
            selectedValid := nodeEnabled[selectedAnchorSlot] and nodeId[selectedAnchorSlot] == selectedAnchorId;
          end if;
          valid := true; seen := fill(false,size(candidatePoint,1));
          for slot in 1:size(previousPoint,1) loop
            feature := insertedFeature[slot];
            slotValid := feature >= 0 and feature <= size(candidatePoint,1)
              and (nextOccupied[slot] == 0.0 or nextOccupied[slot] == 1.0);
            if slotValid then
              if nextOccupied[slot] == 0.0 then
                slotValid := feature == 0;
                localPoint[slot,:] := zeros(3); anchorId[slot] := 0; anchorSlot[slot] := 0;
                clearedCount := clearedCount+1;
              elseif feature > 0 then
                slotValid := selectedValid and feature <= candidateCount and candidateEnabled[feature] == 1.0
                  and not seen[feature];
                seen[feature] := true;
                for axis in 1:3 loop
                  slotValid := slotValid and abs(nextPoint[slot,axis]) <= coordinateLimit
                    and nextPoint[slot,axis] == candidatePoint[feature,axis];
                end for;
                if slotValid then
                  local := transpose(nodeRotation[selectedAnchorSlot,:,:])*(nextPoint[slot,:]-nodePosition[selectedAnchorSlot,:]);
                  reconstructed := nodeRotation[selectedAnchorSlot,:,:]*local+nodePosition[selectedAnchorSlot,:];
                  for axis in 1:3 loop
                    slotValid := slotValid and abs(local[axis]) <= coordinateLimit
                      and abs(reconstructed[axis]-nextPoint[slot,axis]) <= consistencyTolerance;
                  end for;
                  localPoint[slot,:] := local;
                  anchorId[slot] := selectedAnchorId; anchorSlot[slot] := selectedAnchorSlot;
                  assignedCount := assignedCount+1;
                end if;
              else
                owner := previousAnchorSlot[slot];
                slotValid := not reset and previousOccupied[slot] == 1.0
                  and previousAnchorId[slot] >= 1 and previousAnchorId[slot] <= RGBDMapAnchors.identifierLimit
                  and owner >= 1 and owner <= size(nodeEnabled,1);
                for axis in 1:3 loop
                  slotValid := slotValid and abs(previousLocalPoint[slot,axis]) <= coordinateLimit
                    and abs(previousPoint[slot,axis]) <= coordinateLimit
                    and nextPoint[slot,axis] == previousPoint[slot,axis];
                end for;
                if slotValid then
                  slotValid := nodeEnabled[owner] and nodeId[owner] == previousAnchorId[slot];
                  if slotValid then
                    reconstructed := nodeRotation[owner,:,:]*previousLocalPoint[slot,:]+nodePosition[owner,:];
                    for axis in 1:3 loop
                      slotValid := slotValid and abs(reconstructed[axis]-previousPoint[slot,axis]) <= consistencyTolerance;
                    end for;
                    retainedCount := retainedCount+1;
                  end if;
                end if;
              end if;
            end if;
            valid := valid and slotValid;
          end for;
          accepted := valid; rejectionReason := if valid then 0 else 5;
          if accepted then nextGeneration := generation; end if;
        end if;
      end if;
    end if;
  end if;
  if not accepted then
    localPoint := previousLocalPoint; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
    nextGeneration := previousGeneration; assignedCount := 0; retainedCount := 0; clearedCount := 0;
  end if;
end AssignLandmarkAnchors;
