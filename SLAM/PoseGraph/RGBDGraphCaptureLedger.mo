within SLAM.PoseGraph;
// Durable capture birth identities. Catalog ID, camera epoch and accepted
// processing-step sequence are separate clocks; none is inferred from another.
package RGBDGraphCaptureLedger
  import RGBDKeyframes = SLAM.LoopClosure.RGBDKeyframes;

  constant Integer capacity = RGBDKeyframes.keyframeCapacity;
  constant Integer identifierLimit = RGBDKeyframes.identifierLimit;

  record State
    Integer generation; Integer sourceRevision; Integer catalogNextId; Integer lastStep;
    Integer ids[capacity]; Integer epochs[capacity]; Integer sequences[capacity];
    Real times[capacity];
  end State;

  function Empty
    input Integer generation; input Integer sourceRevision;
    output State result;
  algorithm
    result.generation := generation; result.sourceRevision := sourceRevision;
    result.catalogNextId := 1; result.lastStep := 0;
    result.ids := fill(0,capacity); result.epochs := fill(-1,capacity);
    result.sequences := fill(0,capacity); result.times := zeros(capacity);
  end Empty;

  function Valid
    input State ledger; input RGBDKeyframes.Catalog catalog;
    input Integer sourceRevision; input Integer step;
    output Boolean valid;
  protected Integer oldest; Integer slot; Integer previousSequence;
  algorithm
    valid := RGBDKeyframes.ValidHeader(catalog)
      and ledger.generation == catalog.generation
      and sourceRevision >= 1 and sourceRevision <= identifierLimit
      and ledger.sourceRevision == sourceRevision
      and step >= 0 and step < identifierLimit and ledger.lastStep == step
      and ledger.catalogNextId == catalog.nextId;
    oldest := 1; slot := 1; previousSequence := 0;
    if valid then
      oldest := max(1,catalog.nextId-capacity);
      for node in 1:capacity loop
        if node <= min(capacity,catalog.nextId-1) then
          slot := mod(oldest+node-2,capacity)+1;
          valid := valid and ledger.ids[slot] == catalog.ids[slot]
            and ledger.epochs[slot] == catalog.epochs[slot]
            and ledger.times[slot] == catalog.imageTimes[slot]
            and ledger.sequences[slot] > previousSequence
            and ledger.sequences[slot] <= step;
          previousSequence := ledger.sequences[slot];
        end if;
      end for;
    end if;
  end Valid;

  function Advance
    input State previous;
    input RGBDKeyframes.Catalog previousCatalog;
    input RGBDKeyframes.Catalog nextCatalog;
    input Integer sourceRevision;
    input Integer nextStep "Accepted localization publication's actual next.steps";
    input Boolean publicationAccepted;
    input Boolean requested;
    output State result; output Boolean accepted; output Integer reason;
  protected
    State candidate; Boolean valid; Boolean captured; Integer storedSlot;
  algorithm
    result := previous; accepted := false; reason := 1;
    if requested then
      reason := 2;
      valid := publicationAccepted and Valid(previous,previousCatalog,sourceRevision,previous.lastStep)
        and previous.lastStep < identifierLimit-1 and nextStep == previous.lastStep+1
        and RGBDKeyframes.ValidHeader(nextCatalog)
        and nextCatalog.generation == previousCatalog.generation
        and nextCatalog.vocabularyVersion == previousCatalog.vocabularyVersion;
      captured := false; storedSlot := 1;
      if valid then
        reason := 3;
        valid := nextCatalog.nextId == previousCatalog.nextId
          or (previousCatalog.nextId < identifierLimit and nextCatalog.nextId == previousCatalog.nextId+1);
        captured := nextCatalog.nextId == previousCatalog.nextId+1;
        storedSlot := previousCatalog.nextSlot;
        if not captured then
          valid := valid and nextCatalog.lastEpoch == previousCatalog.lastEpoch
            and nextCatalog.lastTime == previousCatalog.lastTime;
        else
          valid := valid and nextCatalog.ids[storedSlot] == previousCatalog.nextId
            and nextCatalog.epochs[storedSlot] > previousCatalog.lastEpoch
            and (previousCatalog.nextId == 1 or nextCatalog.imageTimes[storedSlot] > previousCatalog.lastTime);
        end if;
        for slot in 1:capacity loop
          if not captured or slot <> storedSlot then
            valid := valid and nextCatalog.occupied[slot] == previousCatalog.occupied[slot];
            if previousCatalog.occupied[slot] then
              valid := valid and nextCatalog.ids[slot] == previousCatalog.ids[slot]
                and nextCatalog.epochs[slot] == previousCatalog.epochs[slot]
                and nextCatalog.imageTimes[slot] == previousCatalog.imageTimes[slot];
            end if;
          end if;
        end for;
      end if;
      if valid then
        candidate := previous; candidate.lastStep := nextStep;
        candidate.catalogNextId := nextCatalog.nextId;
        if captured then
          candidate.ids[storedSlot] := nextCatalog.ids[storedSlot];
          candidate.epochs[storedSlot] := nextCatalog.epochs[storedSlot];
          candidate.times[storedSlot] := nextCatalog.imageTimes[storedSlot];
          candidate.sequences[storedSlot] := nextStep;
        end if;
        reason := 4; valid := Valid(candidate,nextCatalog,sourceRevision,nextStep);
        if valid then result := candidate; accepted := true; reason := 0; end if;
      end if;
    end if;
  end Advance;
end RGBDGraphCaptureLedger;
