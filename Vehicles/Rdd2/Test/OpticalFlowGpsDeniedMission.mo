within Vehicles.Rdd2.Test;

model OpticalFlowGpsDeniedMission
  "GPS-aided box mission with a mid-flight GPS outage carried by optical flow"
  parameter Real cruiseAltitude_m = 2.0;
  parameter Real boxSide_m = 4.0;

  extends Vehicles.Rdd2.WaypointVehicleSystem(
    useGlobalWaypoints = false,
    navigationSource = 1,
    fuseOpticalFlowWithGps = true,
    gpsDeniedWindow = true,
    gpsDeniedStart_s = 10.0,
    gpsDeniedEnd_s = 20.0,
    estimatorInitialPositionWorldEnu_m = {0.08, -0.06, 0.04},
    localRoute = [
      0.0,       0.0,       0.0;
      0.0,       0.0,       cruiseAltitude_m;
      boxSide_m, 0.0,       cruiseAltitude_m;
      boxSide_m, boxSide_m, cruiseAltitude_m;
      0.0,       boxSide_m, cruiseAltitude_m;
      0.0,       0.0,       cruiseAltitude_m;
      0.0,       0.0,       0.3;
      0.0,       0.0,       0.1]);

  annotation(
    experiment(
      StartTime = 0.0,
      StopTime = 45.0,
      Tolerance = 1.0e-8,
      Interval = 0.005),
    Documentation(info = "<html>
      <p>Flies the box on the GPS-aided inertial estimate
      (<code>navigationSource&nbsp;=&nbsp;1</code>) but withholds the GPS stream
      for a ten-second window, <code>[10&nbsp;s,&nbsp;20&nbsp;s]</code>, during
      the cruise. Optical flow is fused alongside GPS
      (<code>fuseOpticalFlowWithGps&nbsp;=&nbsp;true</code>), so when GPS drops
      out the horizontal estimate is carried by planar body-velocity flow aiding
      over the textured floor rather than by unaided inertial coasting. This is
      the flow-dominant, discriminating scenario the benchmark wants: it
      separates estimators exactly on their optical-flow fusion, which is where
      the ESKF and UKF are exercised and where a Betaflight-class controller,
      whose firmware has no flow fusion at all, cannot follow, so a swapped
      controller would coast open-loop through the same window.</p>
      <p>The barometer and magnetometer remain available throughout, so the
      outage stresses the horizontal channel specifically. Comparing this
      mission against a variant with <code>gpsDeniedWindow&nbsp;=&nbsp;false</code>
      isolates the drift the GPS outage would produce without the flow fallback.</p>
    </html>"));
end OpticalFlowGpsDeniedMission;
