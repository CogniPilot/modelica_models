within Estimation.FusionHorizon;

block HorizonEstimator
  "One fusion horizon, any strapdown filter, an inertial-rate published state"

  replaceable block FilterModel = Estimation.StrapdownINS.ESKF.Estimator
    constrainedby Estimation.StrapdownINS.PartialEstimator
    "The filter that runs AT the horizon. It is swapped by redeclaration and
     nothing else in this block changes: the buffer, the predictor, the
     re-base, and the published boundary are shared, which is what makes a
     comparison between two filters a comparison of the filters";

  parameter Real samplePeriod(unit = "s", min = 1.0e-9) = 0.00125
    "Inertial and output-predictor tick";
  parameter Real fusionPeriod_s(unit = "s", min = 1.0e-9) = 0.01
    "Filter release interval, one step per composed horizon packet";
  parameter Real fusionHorizon_s(unit = "s", min = 0.0) = 0.2;
  parameter Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  parameter Boolean useFirstOrderHold = true;
  parameter Real initialPositionWorldEnu_m[3] = zeros(3);
  parameter Real initialVelocityWorldEnu_m_s[3] = zeros(3);
  parameter Real initialQuaternionWorldBody[4] = {1.0, 0.0, 0.0, 0.0};
  parameter Real initialGyroscopeBiasBodyFlu_rad_s[3] = zeros(3);
  parameter Real initialAccelerometerBiasBodyFlu_m_s2[3] = zeros(3);

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

  FilterModel filter(
    samplePeriod=fusionPeriod_s,
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
    filterShiftedHeld := filter.status.correctionOutcome ==
      Estimation.StrapdownINS.CorrectionAccepted;
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

  filter.reset = reset;
  // Whole-record equalities rather than connect equations. The aiding
  // streams pass straight through to the filter unchanged, and a connection
  // set whose only members are one outer input and one inner input leaves
  // OpenModelica unable to sort the result against the clocked producers on
  // either side.
  filter.imu = horizon.horizonPacket;
  filter.mocap = mocap;
  filter.gps = gps;
  filter.magnetometer = magnetometer;
  filter.barometer = barometer;
  filter.opticalFlow = opticalFlow;

  annotation(Documentation(info = "<html>
    <p>The composition the architecture exists for. The filter runs AT the
    fusion horizon, where every aiding measurement has already arrived, so it
    never fuses a delayed measurement and never transports a measurement
    Jacobian backwards in time. The state control consumes is the horizon state
    composed with the buffered deltas, republished at the inertial rate.</p>
    <p><b>What this replaces.</b> Measurement-age alignment of the nominal state
    (<code>Estimation.StrapdownINS.ESKF.retrodict</code>, one held IMU sample
    over ages up to 100 ms) and the delay-transport factor of the measurement
    Jacobian. What remains is the undelayed Jacobian itself, which is the half
    that has been mechanically verified. Sensor transport latency is not
    removed by anything here; it is where the horizon length comes from.</p>
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
