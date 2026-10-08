within SLAM.Mapping;
// Landmark coordinates stay local to stable keyframe identities. A graph
// correction is a proposal until every persistent owner commits it together.
package RGBDMapAnchors
  constant Integer imageHeight = 90;
  constant Integer imageWidth = 160;
  constant Integer mapCapacity = imageHeight*imageWidth;
  constant Integer keyframeCapacity = 128;
  constant Integer dimension = 3;
  constant Integer identifierLimit = 1000000000;

  function ProperRotation
    input Real rotation[dimension,dimension];
    output Boolean valid;
  protected
    Real gram[dimension,dimension]; Real determinant;
  algorithm
    valid := true;
    for row in 1:dimension loop
      for column in 1:dimension loop
        valid := valid and abs(rotation[row,column]) <= 1.000001;
      end for;
    end for;
    if valid then
      gram := transpose(rotation)*rotation;
      for row in 1:dimension loop
        for column in 1:dimension loop
          valid := valid and abs(gram[row,column]-(if row == column then 1.0 else 0.0)) <= 1e-6;
        end for;
      end for;
      determinant := rotation[1,1]*(rotation[2,2]*rotation[3,3]-rotation[2,3]*rotation[3,2])
        -rotation[1,2]*(rotation[2,1]*rotation[3,3]-rotation[2,3]*rotation[3,1])
        +rotation[1,3]*(rotation[2,1]*rotation[3,2]-rotation[2,2]*rotation[3,1]);
      valid := valid and abs(determinant-1.0) <= 1e-6;
    end if;
  end ProperRotation;

  // Catalog/graph slots never relocate within a generation. Reuse changes
  // the monotone identity; a cached slot alone cannot keep an anchor alive.
  function ValidNodes
    input Boolean enabled[:];
    input Integer ids[size(enabled,1)];
    input Real position[size(enabled,1),dimension];
    input Real rotation[size(enabled,1),dimension,dimension];
    input Real coordinateLimit;
    output Boolean valid;
  algorithm
    valid := true;
    for node in 1:size(enabled,1) loop
      if enabled[node] then
        valid := valid and ids[node] >= 1 and ids[node] <= identifierLimit
          and ProperRotation(rotation[node,:,:]);
        for axis in 1:dimension loop
          valid := valid and abs(position[node,axis]) <= coordinateLimit;
        end for;
        for other in 1:node-1 loop
          if enabled[other] then valid := valid and ids[node] <> ids[other]; end if;
        end for;
      end if;
    end for;
  end ValidNodes;

  function Reproject
    input Real previousPoint[:,dimension];
    input Real previousLocalPoint[size(previousPoint,1),dimension];
    input Real previousOccupied[size(previousPoint,1)];
    input Integer previousAnchorId[size(previousPoint,1)];
    input Integer previousAnchorSlot[size(previousPoint,1)];
    input Integer mapGeneration; input Integer previousRevision;
    input Boolean nodeEnabled[:];
    input Integer nodeId[size(nodeEnabled,1)];
    input Real nodePosition[size(nodeEnabled,1),dimension];
    input Real nodeRotation[size(nodeEnabled,1),dimension,dimension];
    input Integer graphGeneration; input Integer graphRevision;
    input Boolean graphAccepted; input Boolean requested;
    input Real coordinateLimit;
    output Real point[size(previousPoint,1),dimension];
    output Real localPoint[size(previousPoint,1),dimension];
    output Real occupied[size(previousPoint,1)];
    output Integer anchorId[size(previousPoint,1)];
    output Integer anchorSlot[size(previousPoint,1)];
    output Integer nextRevision;
    output Boolean accepted;
    output Integer rejectionReason;
    output Integer projectedCount; output Integer prunedCount;
  protected
    Boolean configuration; Boolean stateValid; Boolean slotValid;
    Boolean coordinatesValid; Boolean retained;
    Integer owner; Real candidate[dimension];
  algorithm
    point := previousPoint; localPoint := previousLocalPoint;
    occupied := previousOccupied; anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
    nextRevision := previousRevision; accepted := false; rejectionReason := 1;
    projectedCount := 0; prunedCount := 0;
    if requested then
      configuration := size(previousPoint,1) >= 1 and size(previousPoint,1) <= mapCapacity
        and size(nodeEnabled,1) >= 1 and size(nodeEnabled,1) <= keyframeCapacity
        and coordinateLimit > 0.0 and coordinateLimit <= 1e6
        and mapGeneration >= 1 and mapGeneration <= identifierLimit and graphGeneration == mapGeneration
        and previousRevision >= 0 and previousRevision < identifierLimit and graphRevision == previousRevision+1;
      rejectionReason := 2;
      if configuration then
        rejectionReason := 3;
        if graphAccepted then
          rejectionReason := 4;
          if ValidNodes(nodeEnabled,nodeId,nodePosition,nodeRotation,coordinateLimit) then
            stateValid := true; coordinatesValid := true;
            for slot in 1:size(previousPoint,1) loop
              if previousOccupied[slot] == 0.0 then
                point[slot,:] := zeros(dimension); localPoint[slot,:] := zeros(dimension);
                occupied[slot] := 0.0; anchorId[slot] := 0; anchorSlot[slot] := 0;
              elseif previousOccupied[slot] == 1.0 then
                owner := previousAnchorSlot[slot];
                slotValid := previousAnchorId[slot] >= 1 and previousAnchorId[slot] <= identifierLimit
                  and owner >= 1 and owner <= size(nodeEnabled,1);
                for axis in 1:dimension loop
                  slotValid := slotValid and abs(previousPoint[slot,axis]) <= coordinateLimit
                    and abs(previousLocalPoint[slot,axis]) <= coordinateLimit;
                end for;
                stateValid := stateValid and slotValid;
                if slotValid then
                  retained := nodeEnabled[owner] and nodeId[owner] == previousAnchorId[slot];
                  if retained then
                    candidate := nodeRotation[owner,:,:]*previousLocalPoint[slot,:]+nodePosition[owner,:];
                    for axis in 1:dimension loop
                      coordinatesValid := coordinatesValid and abs(candidate[axis]) <= coordinateLimit;
                    end for;
                    point[slot,:] := candidate; projectedCount := projectedCount+1;
                  else
                    // Eviction or slot reuse cannot transfer a landmark to a
                    // new keyframe that happens to occupy the same array slot.
                    point[slot,:] := zeros(dimension); localPoint[slot,:] := zeros(dimension);
                    occupied[slot] := 0.0; anchorId[slot] := 0; anchorSlot[slot] := 0;
                    prunedCount := prunedCount+1;
                  end if;
                end if;
              else
                stateValid := false;
              end if;
            end for;
            accepted := stateValid and coordinatesValid;
            rejectionReason := if not stateValid then 5 else if not coordinatesValid then 6 else 0;
            if accepted then nextRevision := graphRevision; end if;
          end if;
        end if;
      end if;
    end if;
    // Late failures preserve all previous fields, including poisoned disabled
    // payload. Canonicalization, pruning and version advancement commit together.
    if not accepted then
      point := previousPoint; localPoint := previousLocalPoint; occupied := previousOccupied;
      anchorId := previousAnchorId; anchorSlot := previousAnchorSlot;
      nextRevision := previousRevision; projectedCount := 0; prunedCount := 0;
    end if;
  end Reproject;
end RGBDMapAnchors;
