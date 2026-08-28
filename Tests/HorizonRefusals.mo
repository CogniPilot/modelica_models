within Tests;

package HorizonRefusals
  "Configurations the fusion horizon must REFUSE, one model each"

  model ZeroHorizon "A horizon shorter than one release window"
    // horizonWindows rounds to zero here, and at zero windows the first
    // release hands over the ring slot the tick has not written yet: a zero
    // span and a zero quaternion. The consumer divides the rotation increment
    // by that span, so the packet reaches the filter as a not-a-number. This
    // model exists to prove the block refuses rather than publishes it.
    extends Tests.HorizonRefusals.Driver(fusionHorizon_s=0.0);
  end ZeroHorizon;

  model FractionalRelease
    "A release period that is not a whole number of inertial ticks"
    // deltasPerFusion is formed by rounding, so 0.009 against a 1.25 ms tick
    // would quietly have become SEVEN ticks, a release period of 8.75 ms, and
    // every epoch the block published would have been wrong by 0.25 ms per
    // release with nothing reporting it. The horizon is kept an exact multiple
    // of this release period so that this model trips the RELEASE assertion
    // and only that one.
    extends Tests.HorizonRefusals.Driver(
      fusionPeriod_s=0.009,
      fusionHorizon_s=0.045);
  end FractionalRelease;

  model FractionalHorizon
    "A horizon that is not a whole number of release windows"
    // horizonWindows is formed by the same rounding, so 0.055 would have
    // become five windows and the fusion instant would have stood half a
    // window from where the parameter says it stands.
    extends Tests.HorizonRefusals.Driver(fusionHorizon_s=0.055);
  end FractionalHorizon;

  model FusionRateOverBudget
    "A correction rate equal to the fusion rate, which is what a delayed
     horizon actually produces"
    // THE FINDING THIS MODEL PINS. On the live-edge path the predictor's fold
    // budget was charged a nominal 5 Hz of accepted corrections, documented as
    // covering a 5 Hz GPS fix rate. Behind the delayed-measurement queues that
    // number is wrong in kind rather than in size: every measurement the
    // horizon reaches is ripe, the filter accepts at most one per tick and can
    // accept on every tick, and this vehicle's aiding set offers a candidate on
    // essentially every one. The shifted-instant rate is therefore the FUSION
    // rate, 100 Hz at the flight lattice.
    //
    // Against a measured fold budget of 7.3 Hz that is a fourteenfold overrun,
    // and the block refuses it. Composed into Estimation.FusionHorizon
    // HorizonEstimator the same refusal fires from the forwarded
    // correctionRateBudget_hz, but no model holding that block can be built by
    // any tool in this tree, so the refusal is demonstrated here on the
    // predictor alone where it CAN be built and run.
    //
    // The budget is a property of the generated code and not of the
    // architecture: the fold costs about 2500 times its algorithmic content
    // because a record-valued call is materialized once per component. When
    // that lands the budget rises by about the same factor and this
    // configuration stops being refused; see
    // docs/delayed-fusion-horizon-wcet.md.
    extends Tests.HorizonRefusals.Driver(
      fusionHorizon_s=0.05,
      correctionRateBudget_hz=100.0);
  end FusionRateOverBudget;

  model OverBudgetSupervision
    "Supervision parameters that ask for more folds than the target can run"
    // The divergence tolerance that suits a healthy filter's bias moves, at
    // the bias move the block declares it will tolerate, is fifty folds a
    // second against a measured budget of 7.3. Nothing refused this before,
    // and a bound that cannot be honoured is not a bound.
    extends Tests.HorizonRefusals.Driver(
      horizon(maximumPredictorDivergence_rad = 1.0e-3));
  end OverBudgetSupervision;

  partial model Driver "One misconfigured horizon, fed a level stream"
    parameter Real samplePeriod = 0.00125;
    parameter Real fusionPeriod_s = 0.01;
    parameter Real fusionHorizon_s = 0.05;
    parameter Real correctionRateBudget_hz = 5.0;
    Real elapsed_s(start = 0.0, fixed = true)
      "Continuous anchor; not under test";
    Estimation.FusionHorizon.OutputPredictor horizon(
      samplePeriod=samplePeriod,
      fusionPeriod_s=fusionPeriod_s,
      fusionHorizon_s=fusionHorizon_s,
      correctionRateBudget_hz=correctionRateBudget_hz);
  equation
    der(elapsed_s) = 1.0;
    horizon.reset = false;
    horizon.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
    horizon.specificForceMeasuredBodyFlu_m_s2 = {0.0, 0.0, 9.81};
    horizon.horizonStateValid = false;
    horizon.horizonStateShifted = false;
    horizon.horizonPositionWorldEnu_m = zeros(3);
    horizon.horizonVelocityWorldEnu_m_s = zeros(3);
    horizon.horizonQuaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
    horizon.horizonGyroscopeBiasBodyFlu_rad_s = zeros(3);
    horizon.horizonAccelerometerBiasBodyFlu_m_s2 = zeros(3);
    annotation(experiment(StartTime=0.0, StopTime=0.02,
      Tolerance=1.0e-8, Interval=0.001));
  end Driver;

  annotation(Documentation(info="<html>
    <p>NEGATIVE tests. <code>Tests/run-horizon.mos</code> builds and runs each
    of these and requires it to FAIL: the rate lattice, the horizon length and
    the supervision budget are preconditions of the block. The first three used
    to be rounded silently and the fourth was not checked at all, and a
    precondition that is only written in a comment is not one.</p>
    <p>They are a package of top-level models rather than components of
    <code>Tests.All</code> for the obvious reason: a model that must not build
    cannot live inside a model that must.</p>
  </html>"));
end HorizonRefusals;
