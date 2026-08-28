within Tests;

model HorizonEstimatorWiring
  "The same fusion horizon carries an ESKF and a manifold UKF unchanged"

  model Harness "One filter behind the shared horizon"
    // The filter is chosen by modifying this component from a specialising
    // model below, not by a replaceable slot on the harness that forwards into
    // the horizon. Forwarding one replaceable class into another cannot be
    // checked against the constraining type by the compiler this corpus is
    // pinned to, and a redeclaration that names the filter directly is both
    // accepted and easier to read.
    Estimation.FusionHorizon.HorizonEstimator estimator(
      samplePeriod=0.00125,
      fusionPeriod_s=0.01,
      fusionHorizon_s=0.05);
  equation
    // A vehicle at rest, level, at the local origin, with motion capture
    // delivered AT the horizon. Its age is therefore zero and nothing is
    // retrodicted, which is the whole point of fusing at the horizon.
    estimator.reset = false;
    estimator.angularVelocityMeasuredBodyFlu_rad_s = zeros(3);
    estimator.specificForceMeasuredBodyFlu_m_s2 = {0.0, 0.0, 9.81};
    estimator.mocap.valid = true;
    estimator.mocap.fresh = true;
    estimator.mocap.timestamp_s = time - estimator.fusionHorizon_s;
    estimator.mocap.positionWorldEnu_m = zeros(3);
    estimator.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
    estimator.mocap.positionCovarianceWorld_m2 = identity(3) * 1.0e-4;
    estimator.mocap.attitudeCovarianceBody_rad2 = identity(3) * 1.0e-4;
    estimator.gps.valid = false;
    estimator.gps.fresh = false;
    estimator.gps.positionValid = false;
    estimator.gps.velocityValid = false;
    estimator.gps.timestamp_s = time;
    estimator.gps.geodetic_deg_m = zeros(3);
    estimator.gps.positionWorldEnu_m = zeros(3);
    estimator.gps.velocityWorldEnu_m_s = zeros(3);
    estimator.gps.positionCovarianceWorld_m2 = identity(3);
    estimator.gps.velocityCovarianceWorld_m2_s2 = identity(3);
    estimator.magnetometer.valid = false;
    estimator.magnetometer.fresh = false;
    estimator.magnetometer.timestamp_s = time;
    estimator.magnetometer.magneticFieldBodyFlu_T =
      estimator.filter.localMagneticFieldWorldEnu_T;
    estimator.magnetometer.covarianceBody_T2 = identity(3) * 1.0e-12;
    estimator.barometer.valid = false;
    estimator.barometer.fresh = false;
    estimator.barometer.timestamp_s = time;
    estimator.barometer.altitudeWorldEnu_m = 0.0;
    estimator.barometer.variance_m2 = 1.0;
    estimator.opticalFlow.valid = false;
    estimator.opticalFlow.fresh = false;
    estimator.opticalFlow.timestamp_s = time;
    estimator.opticalFlow.integratedLineOfSight_rad = zeros(2);
    estimator.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
    estimator.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
    estimator.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
    estimator.opticalFlow.integrationTime_s = 0.01;
    estimator.opticalFlow.groundDistance_m = 1.0;
    estimator.opticalFlow.groundDistanceVariance_m2 = 0.01;
    estimator.opticalFlow.quality = 1.0;
  end Harness;

  // The generality claim, exercised rather than asserted in prose. Nothing in
  // Estimation.FusionHorizon changes across this line: the ring, the
  // composition, the re-base, and the published boundary are the same code and
  // only the block behind the horizon differs. If the interface had leaked an
  // error-state, a covariance, or a sigma point, this redeclaration would not
  // translate.
  model UkfHarness "The same harness with the manifold UKF behind the horizon"
    extends Harness(estimator(
      redeclare block FilterModel = Estimation.StrapdownINS.UKF.Estimator));
  end UkfHarness;

  Harness eskf;
  UkfHarness ukf;

  annotation(experiment(StartTime=0.0, StopTime=0.1,
    Tolerance=1.0e-8, Interval=0.00125),
    Documentation(info="<html>
    <p>Translated by OpenModelica; neither simulated by OpenModelica nor lowered
    by Rumoca, and both limitations are upstream of this package. A model containing a bare
    <code>Estimation.StrapdownINS.PartialEstimator</code> cannot be built by
    OpenModelica at all -- an independent subset of the flattened estimator is
    reported over-determined by nineteen equations -- and the existing
    <code>Tests.StrapdownEstimatorInterfaceTests</code> fails identically on an
    untouched tree, which is why it too is compiled and never simulated. That
    limitation is upstream of this package and is recorded here so the next
    reader does not mistake a translation-only gate for a design choice.</p>
    <p>Rumoca reports the composed model unbalanced by twenty-eight equations,
    fourteen per harness. That is the same class of imbalance OpenModelica
    reports for a bare estimator, and it appears only when a filter is composed
    with something else; the horizon block on its own lowers all the way to
    galec-production, which is the gate <code>tools/ci.py</code> runs on it.
    The generality claim is therefore carried at translation here and by the
    algebraic and time-domain suites elsewhere, and pinning down the composed
    imbalance is follow-up work rather than something to hide behind a
    weaker test.</p>
    </html>"));
end HorizonEstimatorWiring;
