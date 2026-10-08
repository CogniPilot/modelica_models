within SLAM.LoopClosure;
// Selection only: every camera observation still passes the processing barrier.
// The estimated pose determines motion; sensor identity never comes from time.
package RGBDKeyframePolicy
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;
  import RGBDMapAnchors = SLAM.Mapping.RGBDMapAnchors;

  constant Real pi = 3.141592653589793;
  record Decision
    Boolean valid;
    Boolean captureRequested;
    Integer reason "0 capture; 1 idle; 2 configuration; 3 binding/geometry; 4 pose; 5 features; 6 cadence; 7 motion; 8 consumed image";
    Integer enabledCount;
    Real elapsed;
    Real translationSquared;
    Real rotationCosine;
  end Decision;

  function Select
    input RGBDKeyframes.Catalog catalog;
    input RGBDKeyframes.Frame measurement;
    input Boolean imageFresh "Fresh catalog acquisition receipt; separate from Schmidt independent-noise eligibility";
    input Real poseAccepted;
    input Boolean requested;
    input Real minimumInterval = 0.5;
    input Real maximumInterval = 2.0;
    input Real translationThreshold = 0.6;
    input Real rotationThreshold = 0.25 "Radians";
    input Integer minimumFeatures = 12;
    input Real coordinateLimit = 1e6;
    output Decision result;
  protected
    Boolean valid;
    Integer latestSlot;
    Real difference[RGBDKeyframes.dimension];
    Real relativeRotation[RGBDKeyframes.dimension,RGBDKeyframes.dimension];
  algorithm
    result.valid := false; result.captureRequested := false; result.reason := 1;
    result.enabledCount := 0; result.elapsed := 0.0;
    result.translationSquared := 0.0; result.rotationCosine := 1.0;
    if requested then
      result.reason := 2;
      valid := minimumInterval > 0 and minimumInterval <= 1e4
        and maximumInterval >= minimumInterval and maximumInterval <= 1e4
        and translationThreshold >= 0 and translationThreshold <= 1e4
        and rotationThreshold >= 0 and rotationThreshold <= pi
        and minimumFeatures >= 8 and minimumFeatures <= RGBDKeyframes.featureCapacity
        and coordinateLimit > 0 and coordinateLimit <= 1e6;
      if valid then
        result.reason := 3;
        valid := RGBDKeyframes.ValidHeader(catalog)
          and catalog.nextId < RGBDKeyframes.identifierLimit
          and measurement.generation == catalog.generation
          and measurement.vocabularyVersion == catalog.vocabularyVersion
          and measurement.id == catalog.nextId
          and measurement.epoch > catalog.lastEpoch and measurement.epoch <= RGBDKeyframes.identifierLimit
          and measurement.imageTime >= 0 and measurement.imageTime <= 1e9
          and (catalog.nextId == 1 or measurement.imageTime > catalog.lastTime)
          and measurement.count >= 0 and measurement.count <= RGBDKeyframes.featureCapacity
          and RGBDMapAnchors.ProperRotation(measurement.bodyRotation);
        for axis in 1:RGBDKeyframes.dimension loop
          valid := valid and abs(measurement.bodyPosition[axis]) <= coordinateLimit;
        end for;
        if valid then
          for feature in 1:RGBDKeyframes.featureCapacity loop
            if measurement.enabled[feature] then
              result.enabledCount := result.enabledCount+1;
              valid := valid and feature <= measurement.count and measurement.opticalPoint[feature,3] > 0;
              for axis in 1:RGBDKeyframes.dimension loop
                valid := valid and abs(measurement.opticalPoint[feature,axis]) <= coordinateLimit;
              end for;
            end if;
          end for;
          if catalog.nextId > 1 then
            latestSlot := mod(catalog.nextId-2,RGBDKeyframes.keyframeCapacity)+1;
            valid := valid and RGBDMapAnchors.ProperRotation(catalog.bodyRotations[latestSlot,:,:]);
            for axis in 1:RGBDKeyframes.dimension loop
              valid := valid and abs(catalog.bodyPositions[latestSlot,axis]) <= coordinateLimit;
            end for;
            if valid then
              difference := measurement.bodyPosition-catalog.bodyPositions[latestSlot,:];
              result.translationSquared := difference*difference;
              relativeRotation := transpose(catalog.bodyRotations[latestSlot,:,:])*measurement.bodyRotation;
              result.rotationCosine := max(-1.0,min(1.0,
                (sum(relativeRotation[axis,axis] for axis in 1:RGBDKeyframes.dimension)-1.0)/2.0));
              result.elapsed := measurement.imageTime-catalog.lastTime;
            end if;
          end if;
          result.valid := valid;
          if valid then
            if not imageFresh then
              result.reason := 8;
            elseif poseAccepted <> 1.0 then
              result.reason := 4;
            elseif result.enabledCount < minimumFeatures then
              result.reason := 5;
            elseif catalog.nextId == 1 then
              result.captureRequested := true; result.reason := 0;
            elseif result.elapsed < minimumInterval then
              result.reason := 6;
            elseif result.elapsed >= maximumInterval
                or result.translationSquared >= translationThreshold*translationThreshold
                or result.rotationCosine <= cos(rotationThreshold) then
              result.captureRequested := true; result.reason := 0;
            else
              result.reason := 7;
            end if;
          end if;
        end if;
      end if;
    end if;
  end Select;
end RGBDKeyframePolicy;
