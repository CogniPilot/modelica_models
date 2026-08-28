within Vehicles.Rdd2.Test;

model LongRangeGpsMission
  "The same two hundred metres on GPS aiding, as the drift control"
  extends LongRangeFlowMission(navigationSource = 1);
  annotation(Documentation(info = "<html>
    <p>Identical plant, route, speed, noise seed and controller; the only
    difference is which source aids the filter. GPS observes position
    absolutely, so its horizontal error must stay bounded no matter how far the
    vehicle flies.</p>
    <p>The pair is what separates flow-mode drift from anything the shared
    model does. A growth curve that appeared in BOTH would not be odometry, it
    would be the plant, the controller or the route; a growth curve that
    appears in neither would mean something is feeding absolute position into
    the flow mode illegitimately, which would be a model defect rather than a
    result.</p>
    </html>"));
end LongRangeGpsMission;
