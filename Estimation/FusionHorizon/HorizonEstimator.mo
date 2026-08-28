within Estimation.FusionHorizon;

block HorizonEstimator
  "One fusion horizon, any strapdown filter, an inertial-rate published state"

  replaceable block FilterModel = Estimation.StrapdownINS.ESKF.Estimator
    constrainedby Estimation.StrapdownINS.PartialEstimator
    "The filter that runs AT the horizon. It is swapped by redeclaration and
     nothing else in this block changes: the buffer, the predictor, the
     re-base, and the published boundary are shared, which is what makes a
     comparison between two filters a comparison of the filters";

  // The rate lattice is STRUCTURAL. Both buffers this block composes -- the
  // delta ring and the five measurement queues -- take their lengths from
  // these three numbers, and a buffer length is an array dimension. It has to
  // carry a value when the code is generated, so it is evaluated at
  // translation rather than left tunable, here as well as inside each
  // sub-block: a tunable parameter forwarded into a structural one is still
  // tunable, and the dimension it feeds becomes unevaluable.
  parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.00125
    "Inertial and output-predictor tick"
    annotation(Evaluate = true);
  parameter Real fusionPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "Filter release interval, one step per composed horizon packet"
    annotation(Evaluate = true);
  parameter Real fusionHorizon_s(unit = "s", min = 0.0) = 0.2
    annotation(Evaluate = true);
  parameter Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  parameter Boolean useFirstOrderHold = true;
  parameter Real initialPositionWorldEnu_m[3] = zeros(3);
  parameter Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  parameter Real initialQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  parameter Real initialGyroscopeBiasBodyFlu_rad_s[3] = zeros(3);
  parameter Real initialAccelerometerBiasBodyFlu_m_s2[3] = zeros(3);
  parameter Real mocapPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    annotation(Evaluate = true);
  parameter Real gpsPeriod_s(unit = "s", min = 1.0e-9) = 0.1
    annotation(Evaluate = true);
  parameter Real magnetometerPeriod_s(unit = "s", min = 1.0e-9) = 0.05
    annotation(Evaluate = true);
  parameter Real barometerPeriod_s(unit = "s", min = 1.0e-9) = 0.02
    annotation(Evaluate = true);
  parameter Real opticalFlowPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "Shortest interval each aiding source is declared to deliver at. It sizes
     that source's delayed-measurement queue and nothing else, and it is
     structural for the reason recorded on samplePeriod"
    annotation(Evaluate = true);
  parameter Real maximumSourceDelay_s(unit = "s", min = 0.0) = 0.11
    "Worst end-to-end age any aiding source is declared to deliver at. The
     horizon must cover it with headroom; see
     Estimation.FusionHorizon.AidingBuffer, which owns the relation and asserts
     it";
  parameter Real horizonJitterMargin_s(unit = "s", min = 0.0) = 0.05;
  parameter Real maximumResidualAge_s(unit = "s", min = 0.0) = fusionPeriod_s
    "How far the fusion instant may stand past a measurement's own timestamp
     and still fuse it. See Estimation.FusionHorizon.AidingBuffer, which owns
     this bound and publishes the residual actually achieved";

  input Boolean reset;
  input Real angularVelocityMeasuredBodyFlu_rad_s[3](each unit = "rad/s");
  input Real specificForceMeasuredBodyFlu_m_s2[3](each unit = "m/s2");
  Avionics.MocapSampleInput mocap;
  Avionics.GpsSampleInput gps;
  Avionics.MagnetometerSampleInput magnetometer;
  Avionics.BarometerSampleInput barometer;
  Avionics.OpticalFlowSampleInput opticalFlow;

  discrete Avionics.NavigationEstimateOutput estimate
    "The state at NOW, republished every inertial tick";
  discrete output Boolean horizonReady(start = false, fixed = true);
  discrete output Boolean rebased(start = false, fixed = true);
  discrete output Integer bufferedDeltaCount(start = 0, fixed = true);
  discrete output Boolean biasMoveExceeded(start = false, fixed = true)
    "The filter's bias moved further from the horizon's anchor than the
     first-order Jacobian move is declared good for. A supervision signal, not
     a gate: the state is published as computed";
  discrete output Real worstAidingResidualAge_s(
    unit = "s", start = 0.0, fixed = true)
    "Largest residual any aiding measurement has been delivered to the filter
     with. Bounded by maximumResidualAge_s by construction and published so
     the bound is observable rather than argued: it is the whole quantitative
     claim of fusing at a horizon";
  discrete output Boolean aidingRefused(start = false, fixed = true)
    "A measurement was refused at arrival for being later than the horizon,
     discarded at delivery for the same reason, or displaced from a full
     queue, on this tick";
  discrete output Integer aidingRefusedLateCount(start = 0, fixed = true)
    "Measurements the fusion instant had already passed when they arrived. On
     a horizon longer than every source's transport latency this stays zero,
     and a nonzero value is a statement about the horizon length rather than
     about the filter";

  Estimation.FusionHorizon.OutputPredictor horizon(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2,
    useFirstOrderHold=useFirstOrderHold,
    initialGyroscopeBiasAnchorBodyFlu_rad_s=
      initialGyroscopeBiasBodyFlu_rad_s,
    initialAccelerometerBiasAnchorBodyFlu_m_s2=
      initialAccelerometerBiasBodyFlu_m_s2,
    initialPositionWorldEnu_m=initialPositionWorldEnu_m,
    initialVelocityWorldEnu_m_s=initialVelocityWorldEnu_m_s,
    initialQuaternionWorldBody=initialQuaternionWorldBody);

  // THE OTHER HALF OF A DELAYED HORIZON. Moving the filter's epoch back to
  // t - D without holding the aiding back with it does not fuse delayed
  // measurements, it fuses measurements from the FUTURE: every sensor packet
  // is stamped ahead of the instant the filter is standing on, and the
  // filter's own timestamp rule rejects a negative age outright. The queues
  // hold each measurement until the fusion instant reaches its timestamp.
  Estimation.FusionHorizon.AidingBuffer aiding(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    mocapPeriod_s=mocapPeriod_s,
    gpsPeriod_s=gpsPeriod_s,
    magnetometerPeriod_s=magnetometerPeriod_s,
    barometerPeriod_s=barometerPeriod_s,
    opticalFlowPeriod_s=opticalFlowPeriod_s,
    maximumSourceDelay_s=maximumSourceDelay_s,
    horizonJitterMargin_s=horizonJitterMargin_s,
    maximumResidualAge_s=maximumResidualAge_s);

  FilterModel filter(
    samplePeriod=fusionPeriod_s,
    // The filter's own timestamp bound IS the queue's residual bound, not the
    // quarter second the live-edge path admits. Behind the queues nothing can
    // reach the filter outside that bound, so this is defence in depth rather
    // than the mechanism; what it buys is that a future change which bypasses
    // a queue is refused by timestamp instead of quietly transporting a
    // measurement Jacobian a quarter of a second to meet the state.
    maximumAidingDelay_s=maximumResidualAge_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2,
    initialPositionWorldEnu_m=initialPositionWorldEnu_m,
    initialVelocityWorldEnu_m_s=initialVelocityWorldEnu_m_s,
    initialQuaternionWorldBody=initialQuaternionWorldBody,
    initialGyroscopeBiasBodyFlu_rad_s=initialGyroscopeBiasBodyFlu_rad_s,
    initialAccelerometerBiasBodyFlu_m_s2=
      initialAccelerometerBiasBodyFlu_m_s2);

protected
  discrete Boolean horizonValid(start = false, fixed = true);
  discrete Boolean horizonShifted(start = false, fixed = true);
  discrete Real horizonPosition_m[3](each start = 0.0, each fixed = true);
  discrete Real horizonVelocity_m_s[3](each start = 0.0, each fixed = true);
  discrete Real horizonQuaternion[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real horizonGyroscopeBias_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real horizonAccelerometerBias_m_s2[3](
    each start = 0.0, each fixed = true);
  discrete Boolean filterValidHeld(start = false, fixed = true);
  discrete Boolean filterShiftedHeld(start = false, fixed = true);
  discrete Integer filterCorrectionCountHeld(start = 0, fixed = true)
    "The accepted-correction count as of the previous inertial tick. The edge
     the re-base fires on is a CHANGE in this number, which is the only
     well-defined edge across the rate change between the filter and here.";
  discrete Real filterPositionHeld_m[3](each start = 0.0, each fixed = true);
  discrete Real filterVelocityHeld_m_s[3](each start = 0.0, each fixed = true);
  discrete Real filterQuaternionHeld[4](
    start = {1.0, 0.0, 0.0, 0.0}, each fixed = true);
  discrete Real filterGyroscopeBiasHeld_rad_s[3](
    each start = 0.0, each fixed = true);
  discrete Real filterAccelerometerBiasHeld_m_s2[3](
    each start = 0.0, each fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    // The buffer produces the packet the filter consumes and the filter
    // produces the horizon state the buffer re-bases onto. Both clauses fire
    // together at every release, so the causal order is stated here rather
    // than left for a scheduler to pick: the horizon reads what the filter
    // published, one inertial tick after it published it. That is the right
    // epoch, not a compromise -- the release advanced the buffer tail to the
    // new fusion instant on the same tick the filter advanced its state to it.
    // Every history read is a pre() inside this clause, so the one tick of
    // delay is written where it happens instead of appearing as a pre() in a
    // continuous equation, which OpenModelica cannot sort against a clocked
    // producer.
    horizonValid := pre(filterValidHeld);
    horizonShifted := pre(filterShiftedHeld);
    horizonPosition_m := pre(filterPositionHeld_m);
    horizonVelocity_m_s := pre(filterVelocityHeld_m_s);
    horizonQuaternion := pre(filterQuaternionHeld);
    horizonGyroscopeBias_rad_s := pre(filterGyroscopeBiasHeld_rad_s);
    horizonAccelerometerBias_m_s2 := pre(filterAccelerometerBiasHeld_m_s2);
  end when;

algorithm
  when sample(0.0, samplePeriod) then
    // A separate clause, and that is the causal split, not a formatting
    // choice. The clause above reads only pre() and feeds the horizon; this one
    // reads the filter, which the horizon feeds. Written as one clause the two
    // halves would sit in a current-value cycle that no scheduler can order.
    filterValidHeld := filter.estimate.valid;
    // EDGE, not level. filter.status.correctionOutcome is a LEVEL: it stands
    // for the whole 100 Hz filter tick, which is eight inertial ticks here, so
    // reading it directly fired a re-base on all eight and turned one accepted
    // correction into eight full folds. The WCET record carries what that did
    // to the correction-rate ceiling.
    //
    // A rising edge on the level would not do either: back-to-back accepted
    // corrections hold the level true across the filter-tick boundary, and the
    // second correction would never reach the predictor. So the boundary
    // carries a monotonic accepted-correction count and the edge is a change
    // in it. Avionics.EstimatorStatus.acceptedCorrectionCount was added for
    // this and its documentation records why the level was the wrong signal.
    filterShiftedHeld := filter.status.acceptedCorrectionCount
      <> pre(filterCorrectionCountHeld);
    filterCorrectionCountHeld := filter.status.acceptedCorrectionCount;
    filterPositionHeld_m := filter.estimate.positionWorldEnu_m;
    filterVelocityHeld_m_s := filter.estimate.velocityWorldEnu_m_s;
    filterQuaternionHeld := filter.estimate.quaternionWorldBody;
    filterGyroscopeBiasHeld_rad_s := filter.gyroscopeBiasBodyFlu_rad_s;
    filterAccelerometerBiasHeld_m_s2 := filter.accelerometerBiasBodyFlu_m_s2;
    (estimate.positionWorldEnu_m,
     estimate.velocityWorldEnu_m_s,
     estimate.accelerationWorldEnu_m_s2,
     estimate.quaternionWorldBody,
     estimate.rotationWorldBody,
     estimate.eulerRpy_rad,
     estimate.angularVelocityWorldEnu_rad_s) :=
      Estimation.FusionHorizon.navigationEstimate(
        Estimation.FusionHorizon.Pose(
          positionWorldEnu_m=horizon.positionWorldEnu_m,
          velocityWorldEnu_m_s=horizon.velocityWorldEnu_m_s,
          quaternionWorldBody=horizon.quaternionWorldBody),
        horizon.angularVelocityBodyFlu_rad_s,
        specificForceMeasuredBodyFlu_m_s2,
        filterAccelerometerBiasHeld_m_s2,
        gravityWorldEnu_m_s2);
    estimate.valid := filter.estimate.valid;
    estimate.timestamp_s := time;
    estimate.angularVelocityBodyFlu_rad_s :=
      horizon.angularVelocityBodyFlu_rad_s;
    horizonReady := horizon.horizonReady;
    rebased := horizon.rebased;
    bufferedDeltaCount := horizon.bufferedDeltaCount;
    biasMoveExceeded := horizon.biasMoveExceeded;
    worstAidingResidualAge_s := aiding.worstDeliveredAge_s;
    aidingRefused := aiding.aidingRefused;
    aidingRefusedLateCount := aiding.refusedLateCount;
  end when;

equation
  horizon.reset = reset;
  horizon.angularVelocityMeasuredBodyFlu_rad_s =
    angularVelocityMeasuredBodyFlu_rad_s;
  horizon.specificForceMeasuredBodyFlu_m_s2 =
    specificForceMeasuredBodyFlu_m_s2;
  horizon.horizonStateValid = horizonValid;
  horizon.horizonStateShifted = horizonShifted;
  horizon.horizonPositionWorldEnu_m = horizonPosition_m;
  horizon.horizonVelocityWorldEnu_m_s = horizonVelocity_m_s;
  horizon.horizonQuaternionWorldBody = horizonQuaternion;
  horizon.horizonGyroscopeBiasBodyFlu_rad_s = horizonGyroscopeBias_rad_s;
  horizon.horizonAccelerometerBiasBodyFlu_m_s2 = horizonAccelerometerBias_m_s2;

  // The queues are clocked with the predictor and pulsed by it. The epoch is
  // the one the predictor stamps on the inertial packet it releases, and it
  // has to be the same number: the two packets the filter consumes on one
  // tick name one fusion instant, which is the entire content of fusing at a
  // horizon. Reading it off the packet rather than recomputing it is what
  // makes that identity structural instead of a coincidence between two
  // arithmetic expressions.
  aiding.reset = reset;
  aiding.horizonValid = horizon.horizonReady;
  aiding.horizonEpoch_s = horizon.horizonPacket.timestamp_s;
  aiding.horizonReleased = horizon.horizonPacket.valid;
  aiding.mocap = mocap;
  aiding.gps = gps;
  aiding.magnetometer = magnetometer;
  aiding.barometer = barometer;
  aiding.opticalFlow = opticalFlow;

  filter.reset = reset;
  // Whole-record equalities rather than connect equations. A connection set
  // whose only members are one outer input and one inner input leaves
  // OpenModelica unable to sort the result against the clocked producers on
  // either side.
  //
  // The aiding streams no longer pass straight through. Each one now reaches
  // the filter from its queue, at the fusion instant its own timestamp names,
  // which is what makes the filter's measurement age a residual inside one
  // release window rather than the whole transport latency of the sensor.
  // EVERY source routes through the same path, mocap included: the
  // asymmetry where mocap was aged like the others and aligned like nothing
  // else closes here by construction rather than by adding a fifth copy of a
  // retrodiction stanza.
  filter.imu = horizon.horizonPacket;
  filter.mocap = aiding.mocapAtHorizon;
  filter.gps = aiding.gpsAtHorizon;
  filter.magnetometer = aiding.magnetometerAtHorizon;
  filter.barometer = aiding.barometerAtHorizon;
  filter.opticalFlow = aiding.opticalFlowAtHorizon;

  annotation(Documentation(info = "<html>
    <p>The composition the architecture exists for. The filter runs AT the
    fusion horizon, where every aiding measurement has already arrived, so it
    never fuses a delayed measurement and never transports a measurement
    Jacobian backwards in time. The state control consumes is the horizon state
    composed with the buffered deltas, republished at the inertial rate.</p>
    <p><b>What this replaces.</b> Measurement-age alignment over the whole
    transport latency of a sensor. <code>Estimation.FusionHorizon.AidingBuffer</code>
    holds every aiding packet until the fusion instant reaches its own
    timestamp, so the filter's measurement age is a residual inside one
    release window instead of the age the packet happened to arrive with. The
    interval <code>Estimation.StrapdownINS.ESKF.retrodict</code> and the
    <code>Phi(-age)</code> Jacobian transport run over therefore falls from
    <code>maximumAidingDelay_s</code>, a quarter of a second, to
    <code>fusionPeriod_s</code>, ten milliseconds. Neither function is removed
    and neither is wrong; the argument is that what they are asked to cover is
    now twenty-five times smaller and is a parameter of the release lattice
    rather than a property of a driver. The residual actually achieved is
    published as <code>worstAidingResidualAge_s</code>. Sensor transport
    latency is not removed by anything here; it is where the horizon length
    comes from.</p>
    <p><b>Both halves are required.</b> The predictor alone moves the filter's
    epoch back to <code>t - D</code> and leaves the aiding at the live edge,
    which does not fuse delayed measurements: every sensor packet is then
    stamped AHEAD of the instant the filter stands on, and a negative age is
    refused by the filter's own timestamp rule. The queues are what make the
    delayed epoch usable.</p>
    <p><b>What is generic and what is not.</b> The filter enters through
    <code>Estimation.StrapdownINS.PartialEstimator</code> and is used only
    through the algorithm-neutral part of that boundary: it is handed an
    <code>Avionics.ImuSample</code> and the aiding streams, and it returns a
    pose, two bias vectors, and a correction outcome. The horizon never reads
    <code>navigationCovarianceLocal</code>, never constructs a tangent vector,
    and never asks for an injection, so an additive-bias ESKF, a manifold UKF,
    and a filter with an entirely different uncertainty representation are
    interchangeable here by redeclaration alone. The buffer, the composition,
    and the re-base are bit-identical across that swap, which is the property
    that makes two filters measured in this harness comparable.</p>
  </html>"));
end HorizonEstimator;
