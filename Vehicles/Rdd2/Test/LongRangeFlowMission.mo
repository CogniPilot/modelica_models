within Vehicles.Rdd2.Test;

model LongRangeFlowMission
  "Two hundred metres of optical-flow-aided flight, to characterize drift"
  extends WaypointMission(
    navigationSource = 2,
    boxSide_m = 50.0,
    nominalSpeed = 4.0);
  annotation(
    experiment(
      StartTime = 0.0,
      StopTime = 70.0,
      Tolerance = 1.0e-8,
      Interval = 0.005),
    Documentation(info = "<html>
      <p>The same box the 4 m qualification flies, opened out to 50 m sides so
      one circuit covers 200 m of ground track, at 4 m/s rather than 1 m/s so
      the run stays tractable. Aiding is the unchanged flow mode: optical flow,
      barometer and magnetometer, with GPS off.</p>
      <p><b>What it is for.</b> Flow plus IMU is odometric. Flow observes
      velocity over height, the barometer bounds altitude and the magnetometer
      bounds heading, but nothing observes horizontal POSITION, so horizontal
      error must grow with distance travelled. The 4 m qualification cannot
      show that -- 16 m of track is too short for the growth to separate from
      the noise floor -- and this mission exists to measure the growth and its
      shape rather than to gate anything.</p>
      <p><b>The range is not limited by the flow model.</b> The simulated
      ground is the infinite plane
      <code>opticalFlowGroundNormalWorldEnu = {0,0,1}</code> at
      <code>opticalFlowGroundPlaneOffset_m = 0</code>, so feature visibility
      depends on height and tilt and not on how far the vehicle has travelled.
      What the model does assume is a flat, level, infinitely textured floor:
      real flow over 200 m would meet terrain relief and texture dropouts that
      this fixture does not contain, so the drift measured here is a floor
      rather than a field expectation.</p>
      <p>Speed matters to the answer and is stated for that reason. Drift that
      accumulates per SAMPLE scales as the square root of distance and shrinks
      as speed rises, while drift from a heading bias is linear in distance and
      indifferent to speed. Comparing this run against the 1 m/s qualification
      is comparing two different points on that trade.</p>
    </html>"));
end LongRangeFlowMission;
