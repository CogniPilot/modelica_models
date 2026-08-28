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

  partial model Driver "One misconfigured horizon, fed a level stream"
    parameter Real samplePeriod = 0.00125;
    parameter Real fusionPeriod_s = 0.01;
    parameter Real fusionHorizon_s = 0.05;
    Real elapsed_s(start = 0.0, fixed = true)
      "Continuous anchor; not under test";
    Estimation.FusionHorizon.OutputPredictor horizon(
      samplePeriod=samplePeriod,
      fusionPeriod_s=fusionPeriod_s,
      fusionHorizon_s=fusionHorizon_s);
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
    <p>NEGATIVE tests. <code>Tests/run-horizon.mos</code> simulates each of
    these and requires the simulation to FAIL: the rate lattice and the horizon
    length are preconditions of the block, they used to be rounded silently,
    and a precondition that is only written in a comment is not one.</p>
    <p>They are a package of top-level models rather than components of
    <code>Tests.All</code> for the obvious reason: a model that must not build
    cannot live inside a model that must.</p>
  </html>"));
end HorizonRefusals;
