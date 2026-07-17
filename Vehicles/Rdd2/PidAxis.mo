within Vehicles.Rdd2;
model PidAxis "Typed view of one RDD2 PID axis"
  extends Controller(
    rcRollUs = 1500.0,
    rcPitchUs = 1500.0,
    rcThrottleUs = 1000.0,
    rcYawUs = 1500.0,
    rcArmUs = 1000.0,
    attitudeRoll = 0.0,
    attitudePitch = 0.0,
    attitudeYaw = 0.0,
    throttle = 0.0,
    throttleInputForCommand = 0.0,
    rateCmdRoll = 0.0,
    rateCmdPitch = 0.0,
    rateCmdYaw = 0.0,
    armed = false
  );
end PidAxis;
