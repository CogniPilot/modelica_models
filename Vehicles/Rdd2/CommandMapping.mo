within Vehicles.Rdd2;
model CommandMapping "Typed view of the RDD2 pilot-command mapping"
  extends Controller(
    setpoint = 0.0,
    measurement = 0.0,
    integrate = 0.0,
    throttle = 0.0,
    rateCmdRoll = 0.0,
    rateCmdPitch = 0.0,
    rateCmdYaw = 0.0
  );
end CommandMapping;
