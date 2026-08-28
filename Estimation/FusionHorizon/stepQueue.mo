within Estimation.FusionHorizon;

function stepQueue
  "One inertial tick of one aiding source's delayed-measurement FIFO"
  input Boolean reset;
  input Real queue[:, :]
    "The FIFO as it stood at the previous tick. Column 1 of every row is the
     measurement timestamp; nothing else in this function knows what any other
     column means, which is what lets one kernel serve all five sources.";
  input Integer head(min = 1) "Slot the next admitted measurement is written to";
  input Integer tail(min = 1) "Slot holding the OLDEST queued measurement";
  input Integer count(min = 0) "Measurements standing in the queue";
  input Boolean arrived
    "A valid, finitely stamped, novel sample is being presented this tick. The
     caller owns that test because it is the one thing here that depends on
     the shape of the source record";
  input Real arrivedRow[size(queue, 2)];
  input Boolean releaseTick
    "The horizon released a window to the filter on this tick, so the fusion
     instant advanced and this is the only tick a measurement may be delivered
     on. Deliveries are PULSED onto the release clock for the same reason the
     inertial packet is: the consumer is the filter, whose clock IS the
     release clock, and the two packets it consumes on one tick must name one
     fusion instant";
  input Boolean horizonValid
    "A fusion instant exists. Before the first release the epoch below is
     still its seed value and nothing may be measured against it";
  input Real horizonEpoch_s(unit = "s") "Timestamp of the fusion instant";
  input Real maximumResidualAge_s(unit = "s", min = 0.0)
    "How far the fusion instant may stand past a measurement's own timestamp
     and still fuse it. This is the ONE bound the whole delayed-aiding
     argument rests on, and it is deliberately one number used in two places:
     a measurement is refused at arrival when the horizon has already passed
     it by more than this, and discarded at delivery on the same test. A
     measurement inside it is aligned by transporting the measurement Jacobian
     over that residual only, which at a horizon whose release period is 10 ms
     is a hundredth of the quarter second the live-edge path transported over";
  input Real epochTolerance_s(unit = "s", min = 0.0)
    "Slack on the ripeness comparison. A measurement stamped at the fusion
     instant itself must be ripe AT it, and single-precision timestamps carried
     by repeated addition do not land on a sensor's timestamp exactly";
  output Integer storeSlot
    "Slot the caller must write storedRow into; zero stores nothing";
  output Real storedRow[size(queue, 2)];
  output Integer nextHead(min = 1);
  output Integer nextTail(min = 1);
  output Integer nextCount(min = 0);
  output Real deliveredRow[size(queue, 2)]
    "The measurement handed to the filter, all zeros when none is";
  output Real deliveredAge_s(unit = "s")
    "How far the fusion instant stands past the delivered measurement's own
     timestamp. Zero when nothing was delivered. This is the residual the
     filter still has to transport its measurement Jacobian over, and it is
     bounded by maximumResidualAge_s by construction";
  output Boolean delivered;
  output Integer arrivalOutcome
    "Estimation.FusionHorizon.Aiding* code for what became of an arrival";
  output Integer deliveryOutcome
    "Estimation.FusionHorizon.Aiding* code for what became of the oldest entry";
protected
  Integer depth;
  Real oldestRow[size(queue, 2)];
  Real oldestAge_s;
  Real arrivedAge_s;
  Boolean ripe;
  Boolean popped;
  Integer tailAfterPop;
  Integer occupancyAfterPop;
  Boolean lateArrival;
  Boolean beforeHorizon;
  Boolean admissible;
  Boolean overflow;
  Integer occupancyAfterDrop;
algorithm
  // The whole tick is one pure function for the same reason
  // Estimation.FusionHorizon.step is: it keeps every queue walk inside a
  // function rather than a when-body loop, which is where the code
  // generator's limits on conditional accumulation and data-dependent trip
  // counts live. It also keeps the block a thin clocked shell that does
  // nothing with the queue but write back the row this returned.
  //
  // The queue is flat Real rows and the caller packs and unpacks at the
  // boundary, which is what foldBuffer does and for the same reason: a FIFO
  // whose capacity, walk length and cost are all fixed at translation is the
  // only kind a flight timing record can be written against. That the current
  // code generator also materializes a record-valued call once per component,
  // and so would charge a record-carrying loop its field count, is a second
  // reason and a temporary one; the shape above is the right shape either way.
  depth := size(queue, 1);

  // ---- 1. deliver the oldest entry the fusion instant has reached ---------
  // Read first and decide second. The read is a fixed-length masked walk
  // whether or not anything is delivered, which is what makes the worst case
  // the measured case.
  oldestRow := Estimation.FusionHorizon.readMeasurement(
    queue, if count > 0 then tail else 0);
  oldestAge_s := horizonEpoch_s - oldestRow[1];
  // RIPE means the fusion instant has REACHED the measurement's own
  // timestamp. That is the entire delayed-fusion contract: at the instant the
  // filter is standing on, this measurement is no longer in the future, so
  // fusing it is fusing a measurement at its own epoch rather than
  // transporting one backwards to meet a state that has moved on.
  ripe := releaseTick and horizonValid and count > 0
    and oldestAge_s >= -epochTolerance_s;
  // A ripe entry LEAVES the queue whether it is fused or discarded. Leaving a
  // stale entry at the head would block every fresher measurement behind it
  // for ever, which converts one late packet into a permanently dead source.
  popped := ripe;
  delivered := ripe and oldestAge_s <= maximumResidualAge_s;
  deliveryOutcome := if not ripe then AidingNoDelivery
    elseif delivered then AidingDeliveredAtHorizon
    else AidingDroppedStale;
  // Written as a branch rather than as a masked multiply by a zero-or-one
  // scalar. The masked form is what readMeasurement and storeMeasurement use,
  // where it buys a fixed-length branch-free WALK; here it is a single
  // assignment and buys nothing, and OpenModelica 1.27 miscompiles it:
  // multiplying a local array by an if-expression on a Boolean and assigning
  // the result to an array OUTPUT of a multiple-output function published
  // zeros, silently, while every scalar output of the same call came out
  // right. Measured before this was written plainly: sixteen deliveries whose
  // reported age was correct at 0.00875 s and whose payload and timestamp
  // were both zero.
  if delivered then
    deliveredRow := oldestRow;
    deliveredAge_s := max(oldestAge_s, 0.0);
  else
    deliveredRow := zeros(size(queue, 2));
    deliveredAge_s := 0.0;
  end if;
  tailAfterPop := if popped then (if tail >= depth then 1 else tail + 1)
    else tail;
  occupancyAfterPop := count - (if popped then 1 else 0);

  // ---- 2. admit the arriving measurement ----------------------------------
  // The pop happens FIRST so a queue that is full can drain on the same tick
  // it is written to. Ordered the other way a full queue would displace an
  // entry it was about to deliver anyway.
  arrivedAge_s := horizonEpoch_s - arrivedRow[1];
  // OLDER THAN THE HORIZON ON ARRIVAL. There is no fusion instant left to
  // fuse this at: the filter passed its epoch before it got here. The
  // live-edge path answered this case by transporting the measurement
  // Jacobian back over the whole age, up to a quarter of a second, at a
  // cubic-Taylor truncation error the timing record puts at 12 to 27 percent.
  // This path refuses it and NAMES the refusal instead. A named refusal a
  // supervisor can act on is worth more than a fused measurement nobody can
  // bound the error of.
  lateArrival := arrived and horizonValid
    and arrivedAge_s > maximumResidualAge_s;
  // A MEASUREMENT PRESENTED BEFORE THE FIRST RELEASE is not admitted and is
  // not a refusal. There is no fusion instant yet for its timestamp to be
  // measured against, and the filter has had no inertial packet either, so
  // there was nothing it could have been fused at. Admitting it would queue a
  // whole horizon of samples that are already older than the residual bound by
  // the time the epoch starts moving, and every one of them would then be
  // discarded as stale -- a startup transient reported as a fault, which
  // desensitizes the one signal that says a real source is arriving late.
  //
  // It is left UNCONSUMED rather than swallowed: the caller does not advance
  // its novelty timestamp on this outcome, so a source that holds its packet
  // between pulses has it admitted on the first tick after the horizon
  // becomes real rather than losing it.
  beforeHorizon := arrived and not horizonValid;
  // OVERFLOW REFUSES THE ARRIVAL and keeps what the queue already holds. The
  // reasoning is recorded on AidingRefusedOverflow, and it is the opposite of
  // the answer a live-edge buffer wants: here the oldest entry is the one the
  // fusion instant is about to reach, so displacing it to make room is
  // throwing away the only entry that was about to be usable.
  overflow := arrived and horizonValid and not lateArrival
    and occupancyAfterPop >= depth;
  admissible := arrived and horizonValid and not lateArrival and not overflow;
  occupancyAfterDrop := occupancyAfterPop;
  nextTail := tailAfterPop;
  storeSlot := if admissible then head else 0;
  storedRow := arrivedRow;
  nextHead := if admissible then (if head >= depth then 1 else head + 1)
    else head;
  nextCount := occupancyAfterDrop + (if admissible then 1 else 0);
  arrivalOutcome := if not arrived then AidingNoArrival
    elseif beforeHorizon then AidingBeforeHorizon
    elseif lateArrival then AidingRefusedLate
    elseif overflow then AidingRefusedOverflow
    else AidingQueued;

  // ---- 3. a reset drops the queue ----------------------------------------
  // Every queued measurement describes a state that no longer exists, so none
  // of them may be fused after the reset. Written as an override at the end
  // rather than as a branch around the whole body so the arithmetic above has
  // exactly one form and is exercised on every tick.
  if reset then
    storeSlot := 0;
    nextHead := 1;
    nextTail := 1;
    nextCount := 0;
    deliveredRow := zeros(size(queue, 2));
    deliveredAge_s := 0.0;
    delivered := false;
    arrivalOutcome := AidingNoArrival;
    deliveryOutcome := AidingNoDelivery;
  end if;
end stepQueue;
