within Tests;

package AidingHorizonRefusals
  "Configurations Estimation.FusionHorizon.AidingBuffer must refuse"

  model TightResidual
    "A residual bound tighter than one release window stops all aiding"
    extends Tests.AidingHorizonRefusals.Arm(
      maximumResidualAge_s = 0.5 * fusionPeriod_s);
    annotation(experiment(StartTime=0.0, StopTime=0.05,
      Tolerance=1.0e-8, Interval=0.00125),
      Documentation(info="<html>
      <p>Must NOT run. A measurement ripens on the first release at or after
      its own timestamp, so its residual reaches one release period by
      construction. A bound below that discards every measurement as stale
      while reporting a stale drop, which is a configuration error wearing a
      supervision code: aiding stops and nothing says the configuration was
      the cause.</p>
      </html>"));
  end TightResidual;

  partial model Arm "One buffer, driven, with nothing asserted"
    parameter Real samplePeriod = 0.00125;
    parameter Real fusionPeriod_s = 0.01;
    parameter Real fusionHorizon_s = 0.02;
    parameter Real maximumResidualAge_s = fusionPeriod_s;
    Real elapsed_s(start = 0.0, fixed = true);
    Estimation.FusionHorizon.AidingBuffer buffer(
      samplePeriod=samplePeriod,
      fusionPeriod_s=fusionPeriod_s,
      fusionHorizon_s=fusionHorizon_s,
      maximumResidualAge_s=maximumResidualAge_s);
  equation
    der(elapsed_s) = 1.0;
    buffer.reset = false;
    buffer.horizonValid = false;
    buffer.horizonEpoch_s = 0.0;
    buffer.horizonReleased = false;
    buffer.mocap.valid = false;
    buffer.mocap.fresh = false;
    buffer.mocap.timestamp_s = time;
    buffer.mocap.positionWorldEnu_m = zeros(3);
    buffer.mocap.quaternionWorldBody = {1.0, 0.0, 0.0, 0.0};
    buffer.mocap.positionCovarianceWorld_m2 = identity(3);
    buffer.mocap.attitudeCovarianceBody_rad2 = identity(3);
    buffer.gps.valid = false;
    buffer.gps.fresh = false;
    buffer.gps.positionValid = false;
    buffer.gps.velocityValid = false;
    buffer.gps.timestamp_s = time;
    buffer.gps.geodetic_deg_m = zeros(3);
    buffer.gps.positionWorldEnu_m = zeros(3);
    buffer.gps.velocityWorldEnu_m_s = zeros(3);
    buffer.gps.positionCovarianceWorld_m2 = identity(3);
    buffer.gps.velocityCovarianceWorld_m2_s2 = identity(3);
    buffer.magnetometer.valid = false;
    buffer.magnetometer.fresh = false;
    buffer.magnetometer.timestamp_s = time;
    buffer.magnetometer.magneticFieldBodyFlu_T = zeros(3);
    buffer.magnetometer.covarianceBody_T2 = identity(3);
    buffer.barometer.valid = false;
    buffer.barometer.fresh = false;
    buffer.barometer.timestamp_s = time;
    buffer.barometer.altitudeWorldEnu_m = 0.0;
    buffer.barometer.variance_m2 = 1.0;
    buffer.opticalFlow.valid = false;
    buffer.opticalFlow.fresh = false;
    buffer.opticalFlow.timestamp_s = time;
    buffer.opticalFlow.integratedLineOfSight_rad = zeros(2);
    buffer.opticalFlow.integratedLineOfSightCovariance_rad2 = identity(2);
    buffer.opticalFlow.integratedGyroscopeBodyFlu_rad = zeros(3);
    buffer.opticalFlow.integratedGyroscopeCovariance_rad2 = identity(3);
    buffer.opticalFlow.integrationTime_s = 0.01;
    buffer.opticalFlow.groundDistance_m = 1.0;
    buffer.opticalFlow.groundDistanceVariance_m2 = 0.01;
    buffer.opticalFlow.quality = 1.0;
  end Arm;

  annotation(Documentation(info="<html>
    <p>One NEGATIVE model per precondition, run by
    <code>Tests/run-horizon.mos</code>, where the requirement on each is that
    it does NOT run. A precondition written only in a comment is not one.</p>
    <p>There is one, not five, and the missing four are worth naming. An
    earlier version of the block also asserted that each queue could hold one
    horizon of its own source. With the capacities derived from the declared
    source rates that inequality is an identity, so those assertions could not
    fail for any configuration the block admits, and an assertion that cannot
    fire reads as coverage without being any. They were deleted rather than
    given negative tests that would have had to defeat the derivation to
    provoke them.</p>
    </html>"));
end AidingHorizonRefusals;
