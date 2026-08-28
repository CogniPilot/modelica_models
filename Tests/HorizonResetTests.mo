within Tests;

model HorizonResetTests
  "The packet epoch is re-anchored by a mid-run reset and never drifts from
   the buffer it describes"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Real fusionPeriod_s = 0.01 "100 Hz fusion release";
  constant Real fusionHorizon_s = 0.05 "Five buffered release windows";
  constant Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  constant Real specificForceBodyFlu_m_s2[3] = {0.5, 0.0, 9.81};
  constant Real resetTime_s = 0.2
    "Mid-run, long after the ring has filled and released. This is the case
     that used to strand the epoch: the block seeded the packet timestamp at
     minus one sample period whatever the wall clock said, so after a reset at
     0.2 s the epoch it published was 0.2 s behind the state it described, for
     the rest of the flight. Downstream, aiding aligned by timestamp would
     have been rejected from here on.";
  constant Real refilled_s = 0.28
    "Past the reset plus one horizon plus one release window";

  Real elapsed_s(start = 0.0, fixed = true)
    "Continuous anchor. A model assembled only from clocked blocks has no
     continuous equation at all and OpenModelica index reduction refuses to
     build one. Not under test.";

  Estimation.FusionHorizon.OutputPredictor driven(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2);

  discrete Boolean resetPulse(start = false, fixed = true);
  discrete Boolean resetDone(start = false, fixed = true);
  discrete Real fusionEpochAge_s(start = 0.0, fixed = true);
  discrete Real epochAgainstBuffer_s(start = 0.0, fixed = true);
  discrete Real epochAgainstReset_s(start = 0.0, fixed = true);
  discrete Boolean readyBeforeReset(start = false, fixed = true);
  discrete Boolean readyAfterRefill(start = false, fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    // Half a tick of slack, because the sampled instants are k * samplePeriod
    // in binary floating point and 0.2 need not be one of them to the last
    // bit. One tick of reset and no more.
    resetPulse := time >= resetTime_s - 0.5 * samplePeriod
      and not pre(resetDone);
    resetDone := pre(resetDone) or resetPulse;
  end when;

algorithm
  when sample(0.0, samplePeriod) then
    fusionEpochAge_s := time - driven.horizonPacket.timestamp_s;
    epochAgainstBuffer_s := fusionEpochAge_s
      - driven.bufferedDeltaCount * samplePeriod;
    // On the reset tick the epoch must be re-anchored to THIS tick, one
    // sample back, exactly as it is at power-on.
    epochAgainstReset_s := if resetPulse
      then driven.horizonPacket.timestamp_s - (time - samplePeriod)
      else 0.0;
    readyBeforeReset := pre(readyBeforeReset)
      or (driven.horizonReady and time < resetTime_s - samplePeriod);
    readyAfterRefill := pre(readyAfterRefill)
      or (driven.horizonReady and time > refilled_s);
  end when;

equation
  der(elapsed_s) = 1.0;

  driven.reset = resetPulse;
  driven.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
  driven.specificForceMeasuredBodyFlu_m_s2 = specificForceBodyFlu_m_s2;
  driven.horizonStateValid = false;
  driven.horizonStateShifted = false;
  driven.horizonPositionWorldEnu_m = zeros(3);
  driven.horizonVelocityWorldEnu_m_s = zeros(3);
  driven.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  driven.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
  driven.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  // THE INVARIANT, and it holds on every tick of the run including the reset
  // and the refill behind it: the age of the epoch the block publishes is
  // exactly the number of inertial ticks the buffer is carrying. It is what
  // makes the epoch a statement about the buffer rather than a number that
  // happens to increase. Before the reset was re-anchored this settled at
  // about 3.4 times the bound it is allowed and never came back.
  assert(abs(epochAgainstBuffer_s) < 0.5 * samplePeriod,
    "The published fusion epoch and the buffer span disagree, so the epoch is
     not a statement about the state the buffer carries");
  assert(abs(epochAgainstReset_s) < 0.5 * samplePeriod,
    "A reset did not re-anchor the packet epoch to the tick it happened on, so
     every packet after it is stamped with an epoch that predates the reset");
  // A reset drops the buffer, so readiness must drop with it and come back
  // only when a packet has been released again.
  assert(not (driven.horizonReady and resetPulse),
    "A reset left horizonReady standing, so a consumer would keep standing on
     a fusion instant the buffer no longer holds");
  assert(fusionEpochAge_s <= fusionHorizon_s + fusionPeriod_s
      + 2.0 * samplePeriod or not driven.horizonReady,
    "The fused epoch fell more than one release window behind the horizon");
  assert(readyBeforeReset or time < resetTime_s,
    "The horizon never became ready before the reset, so the reset under test
     did not drop anything");
  assert(readyAfterRefill or time < refilled_s + samplePeriod,
    "The horizon never became ready again after the reset, so the buffer did
     not refill");

  annotation(experiment(StartTime=0.0, StopTime=0.32,
    Tolerance=1.0e-8, Interval=0.001),
    Documentation(info="<html>
    <p>Simulated as a top-level model through
    <code>Tests/run-horizon.mos</code>. The property is the one thing a reset
    can silently break and nothing else in the suite covers: the packet epoch
    is carried, not read off a clock, so the only place it can be anchored to
    wall time is the reset itself.</p>
    </html>"));
end HorizonResetTests;
