within SLAM.Mapping;
// Collision-safe bounded spatial lookup. Storage and all search mathematics are
// Modelica-owned. Rebuild after pruning; inserted points keep their anchor.
package RGBDSpatialIndex
  constant Integer defaultBucketCount = 32768;
  constant Integer maximumBucketCount = 65536;
  constant Integer maximumSlotCount = 1000000;
  constant Real coordinateLimit = 1e6;
  constant Integer maximumIndexedRadius = 4;

  function Cell
    input Real point[3];
    input Real voxelWidth;
    output Integer cell[3];
    output Boolean valid;
  algorithm
    cell := fill(0,3);
    valid := voxelWidth >= 0.001 and voxelWidth <= 10.0;
    for axis in 1:3 loop
      valid := valid and abs(point[axis]) <= coordinateLimit;
    end for;
    if valid then
      for axis in 1:3 loop
        cell[axis] := integer(floor(point[axis]/voxelWidth));
      end for;
    end if;
  end Cell;

  function Bucket
    input Integer cell[3];
    input Integer bucketCount;
    output Integer bucket;
  protected
    Integer hash;
  algorithm
    // The caller certifies 1..maximumBucketCount buckets. Reduce each key
    // before multiplication: the intermediate fits even a signed 32-bit Integer.
    hash := 0;
    for axis in 1:3 loop
      hash := mod(31*hash+mod(cell[axis],bucketCount),bucketCount);
    end for;
    bucket := hash+1;
  end Bucket;

  function Build
    input Real point[:,3];
    input Real occupied[size(point,1)];
    input Real voxelWidth;
    input Integer bucketCount = defaultBucketCount;
    output Integer head[bucketCount];
    output Integer link[size(point,1)];
    output Integer cell[size(point,1),3];
    output Integer freeNext[size(point,1)];
    output Integer firstFree;
    output Boolean valid;
  protected
    Integer slot;
    Integer bucket;
    Boolean pointValid;
  algorithm
    head := fill(0,bucketCount);
    link := fill(0,size(point,1));
    cell := fill(0,size(point,1),3);
    freeNext := fill(0,size(point,1));
    firstFree := 0;
    valid := size(point,1) > 0 and size(point,1) <= maximumSlotCount
      and bucketCount >= 1 and bucketCount <= maximumBucketCount
      and voxelWidth >= 0.001 and voxelWidth <= 10.0;
    if valid then
      // Reverse construction makes both chains and the free list ascending.
      // Storage uses array indices, with no lossy collision replacement.
      // Allocation and buffer reuse are compiler/runtime responsibilities.
      for reverse in 1:size(point,1) loop
        slot := size(point,1)+1-reverse;
        if occupied[slot] == 1.0 then
          (cell[slot,:],pointValid) := Cell(point[slot,:],voxelWidth);
          valid := valid and pointValid;
          if pointValid then
            bucket := Bucket(cell[slot,:],bucketCount);
            link[slot] := head[bucket];
            head[bucket] := slot;
          end if;
        elseif occupied[slot] == 0.0 then
          freeNext[slot] := firstFree;
          firstFree := slot;
        else
          valid := false;
        end if;
      end for;
    end if;
    if not valid then
      head := fill(0,bucketCount);
      link := fill(0,size(point,1));
      cell := fill(0,size(point,1),3);
      freeNext := fill(0,size(point,1));
      firstFree := 0;
    end if;
  end Build;

  function Find
    input Real point[:,3];
    input Real occupied[size(point,1)];
    input Integer head[:];
    input Integer link[size(point,1)];
    input Integer cell[size(point,1),3];
    input Real candidate[3];
    input Real voxelWidth;
    input Real mergeRadius;
    input Boolean searchEnabled;
    output Integer duplicate;
    output Boolean valid;
    output Integer visited "Occupied point checks, for algorithmic profiling";
    output Boolean linearSearch;
  protected
    Integer queryCell[3];
    Integer neighbor[3];
    Integer radius;
    Integer neighborhood;
    Integer node;
    Integer traversed;
    Real difference[3];
    Boolean sameVoxel;
    Boolean matches;
  algorithm
    duplicate := 0;
    visited := 0;
    linearSearch := false;
    valid := true;
    queryCell := fill(0,3);
    neighbor := fill(0,3);
    difference := zeros(3);
    radius := 0;
    neighborhood := 0;
    node := 0;
    traversed := 0;
    sameVoxel := false;
    matches := false;
    if searchEnabled then
      (queryCell,valid) := Cell(candidate,voxelWidth);
      valid := valid and size(point,1) > 0 and size(point,1) <= maximumSlotCount
        and size(head,1) >= 1 and size(head,1) <= maximumBucketCount
        and mergeRadius >= 0.0 and mergeRadius <= 10.0;
      if valid then
        radius := integer(ceil(mergeRadius/voxelWidth));
        // Broad radii remain correct without iterating billions of empty cells.
        linearSearch := radius > maximumIndexedRadius;
        if not linearSearch then
          neighborhood := (2*radius+1)*(2*radius+1)*(2*radius+1);
          linearSearch := neighborhood >= size(point,1)
            or neighborhood >= size(head,1);
        end if;
        if linearSearch then
          for slot in 1:size(point,1) loop
            if occupied[slot] == 1.0 then
              visited := visited+1;
              difference := point[slot,:]-candidate;
              sameVoxel := floor(point[slot,1]/voxelWidth) == queryCell[1]
                and floor(point[slot,2]/voxelWidth) == queryCell[2]
                and floor(point[slot,3]/voxelWidth) == queryCell[3];
              matches := sameVoxel or difference*difference <= mergeRadius^2;
              if matches and duplicate == 0 then
                duplicate := slot;
              end if;
            end if;
          end for;
        else
          for dx in -radius:radius loop
            for dy in -radius:radius loop
              for dz in -radius:radius loop
                neighbor := queryCell+{dx,dy,dz};
                node := head[Bucket(neighbor,size(head,1))];
                traversed := 0;
                valid := valid and node >= 0 and node <= size(point,1);
                while node > 0 and valid loop
                  // Bound corrupt cycles and refuse invalid indices before reads.
                  if node > size(point,1) or traversed >= size(point,1) then
                    valid := false;
                  else
                    traversed := traversed+1;
                    visited := visited+1;
                    valid := occupied[node] == 1.0
                      and link[node] >= 0 and link[node] <= size(point,1);
                    if valid then
                      if cell[node,1] == neighbor[1] and cell[node,2] == neighbor[2]
                        and cell[node,3] == neighbor[3] then
                        difference := point[node,:]-candidate;
                        sameVoxel := neighbor[1] == queryCell[1]
                          and neighbor[2] == queryCell[2] and neighbor[3] == queryCell[3];
                        matches := sameVoxel or difference*difference <= mergeRadius^2;
                        if matches and (duplicate == 0 or node < duplicate) then
                          duplicate := node;
                        end if;
                      end if;
                      node := link[node];
                    end if;
                  end if;
                end while;
              end for;
            end for;
          end for;
        end if;
      end if;
    end if;
    if not valid then
      duplicate := 0;
    end if;
  end Find;
end RGBDSpatialIndex;
