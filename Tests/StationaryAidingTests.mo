within Tests;

model StationaryAidingTests
  "Declared rest fuses alongside live aiding and ends when released"
  extends MultisensorAidingTests(estimator(vehicleAtRest=time < 0.6,
    zeroVelocityVariance_m2_s2=0.0));
algorithm
  when sample(0.005, 0.01) then
    if observedTick > 10 then
      assert(estimator.status.zeroVelocityCorrectionAccepted == (time < 0.6),
        "Declared stationary velocity was starved or remained active in flight");
    end if;
  end when;
end StationaryAidingTests;
