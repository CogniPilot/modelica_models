within Vehicles.Rdd2;
model Mixer "Typed view of the RDD2 four-motor mixer"
  extends Controller(
    setpoint = 0.0,
    measurement = 0.0,
    integrate = 0.0,
    rcRollUs = 1500.0,
    rcPitchUs = 1500.0,
    rcThrottleUs = 1000.0,
    rcYawUs = 1500.0,
    rcArmUs = 1000.0,
    attitudeRoll = 0.0,
    attitudePitch = 0.0,
    attitudeYaw = 0.0,
    throttleInputForCommand = 0.0
  );
end Mixer;
