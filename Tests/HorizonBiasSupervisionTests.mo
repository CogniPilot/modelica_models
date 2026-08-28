within Tests;

model HorizonBiasSupervisionTests
  "The bias move is bounded and reported, and the drift the incremental path
   does not carry is bounded by a forced re-base"

  constant Real samplePeriod = 0.00125 "800 Hz inertial tick";
  constant Real fusionPeriod_s = 0.01 "100 Hz fusion release";
  constant Real fusionHorizon_s = 0.05 "Five buffered release windows";
  constant Real gravityWorldEnu_m_s2[3] = {0.0, 0.0, -9.81};
  constant Real maximumGyroscopeBiasMove_rad_s = 0.05;
  constant Real maximumPredictorDivergence_rad = 1.0e-3;
  constant Real insideBias_rad_s[3] = {0.02, 0.0, 0.0}
    "Inside the declared ball around the anchor, so the move is applied and
     nothing is flagged. 0.02 rad/s against a 1e-3 rad divergence tolerance is
     a forced re-base every 50 ms, which is 40 inertial ticks.";
  constant Real outsideBias_rad_s[3] = {0.1, 0.0, 0.0}
    "Outside it, so biasMoveExceeded stands and the state is published anyway";
  constant Integer reanchorTicks = 40
    "maximumPredictorDivergence_rad / (||db_g|| * samplePeriod) for the inside
     case, derived rather than observed";
  constant Real settled_s = 0.08 "Past the horizon fill and the first release";

  Real elapsed_s(start = 0.0, fixed = true)
    "Continuous anchor. A model assembled only from clocked blocks has no
     continuous equation at all and OpenModelica index reduction refuses to
     build one. Not under test.";

  Estimation.FusionHorizon.OutputPredictor inside(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2,
    maximumGyroscopeBiasMove_rad_s=maximumGyroscopeBiasMove_rad_s,
    maximumPredictorDivergence_rad=maximumPredictorDivergence_rad);
  Estimation.FusionHorizon.OutputPredictor outside(
    samplePeriod=samplePeriod,
    fusionPeriod_s=fusionPeriod_s,
    fusionHorizon_s=fusionHorizon_s,
    gravityWorldEnu_m_s2=gravityWorldEnu_m_s2,
    maximumGyroscopeBiasMove_rad_s=maximumGyroscopeBiasMove_rad_s,
    maximumPredictorDivergence_rad=maximumPredictorDivergence_rad);

  discrete Integer insideTicksSinceRebase(start = 0, fixed = true);
  discrete Integer worstTicksSinceRebase(start = 0, fixed = true);

algorithm
  when sample(0.0, samplePeriod) then
    insideTicksSinceRebase := if inside.rebased then 0
      else pre(insideTicksSinceRebase) + 1;
    worstTicksSinceRebase := if time < settled_s then 0
      else max(pre(worstTicksSinceRebase), insideTicksSinceRebase);
  end when;

equation
  der(elapsed_s) = 1.0;

  // A level, at-rest stream. The property under test is a supervision
  // property, not a tracking one: the horizon pose handed back below is a
  // fixed stand-in rather than a filter's answer, so the predicted state jumps
  // on every forced re-base and that is expected. What is asserted is the
  // FLAG and the CADENCE, and neither depends on the pose being truthful.
  inside.reset = false;
  inside.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
  inside.specificForceMeasuredBodyFlu_m_s2 = {0.0, 0.0, 9.81};
  inside.horizonStateValid = inside.horizonReady;
  inside.horizonStateShifted = false;
  inside.horizonPositionWorldEnu_m = zeros(3);
  inside.horizonVelocityWorldEnu_m_s = zeros(3);
  inside.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  inside.horizonGyroscopeBiasBodyFlu_rad_s = insideBias_rad_s;
  inside.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  outside.reset = false;
  outside.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
  outside.specificForceMeasuredBodyFlu_m_s2 = {0.0, 0.0, 9.81};
  outside.horizonStateValid = outside.horizonReady;
  outside.horizonStateShifted = false;
  outside.horizonPositionWorldEnu_m = zeros(3);
  outside.horizonVelocityWorldEnu_m_s = zeros(3);
  outside.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
  outside.horizonGyroscopeBiasBodyFlu_rad_s = outsideBias_rad_s;
  outside.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);

  // ---- the bound is reported, not enforced --------------------------------
  // The first-order move is good over a stated ball and no further. Outside
  // it the answer is still the best available one, so it is published and
  // flagged; clamping it would put the predictor on a bias nobody estimated
  // and report nothing.
  assert(outside.biasMoveExceeded or time < settled_s,
    "A bias move twice the declared bound was applied without raising
     biasMoveExceeded, so the first-order validity limit is unobservable");
  assert(not inside.biasMoveExceeded,
    "A bias move inside the declared bound raised biasMoveExceeded");

  // ---- the incremental path's drift is bounded ----------------------------
  // Only a re-base carries the bias move onto the buffered window; between
  // re-bases the predictor leaves the filter's own bias at ||db_g||, without
  // bound in the time since the last one. The re-anchor makes that time
  // bounded by construction, and this is the assertion that says so. Reverting
  // it leaves the counter climbing for the whole run.
  // Measured: 41 ticks, against the derived 40.
  assert(worstTicksSinceRebase <= reanchorTicks + 2,
    "The predictor went longer without a re-base than the declared divergence
     tolerance allows, so the drift the incremental path does not carry is
     unbounded in flight time");
  assert(worstTicksSinceRebase >= reanchorTicks - 2 or time < 0.2,
    "The predictor re-based more often than the divergence tolerance asks for,
     so the trigger is not the one this test is measuring");

  annotation(experiment(StartTime=0.0, StopTime=0.3,
    Tolerance=1.0e-8, Interval=0.001),
    Documentation(info="<html>
    <p>Simulated as a top-level model through
    <code>Tests/run-horizon.mos</code>. Two supervision properties that have no
    other home: the bias-move validity bound is REPORTED rather than clamped,
    and the drift the incremental path does not carry is bounded by a forced
    re-base rather than left to grow with flight time.</p>
    </html>"));
end HorizonBiasSupervisionTests;
