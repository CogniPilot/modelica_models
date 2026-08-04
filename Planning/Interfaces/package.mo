within Planning;
package Interfaces "Typed boundaries for deployable planning components"

  connector WaypointPlanInput
    "Fixed-capacity waypoint plan received from a mission-data message"
    parameter Integer capacity(min = 2) = 16;

    input Boolean valid "The message contains a complete candidate plan";
    input Integer sequence "Changes whenever the sender publishes a new plan";
    input Integer waypointCount(min = 0, max = capacity)
      "Number of populated rows";
    input Boolean globalFrame
      "True for geodetic rows; false for local East-North-Up rows";
    input Real originGeodetic[3]
      "Local origin [latitude_deg, longitude_deg, altitude_m]";
    input Real waypoint[capacity, 3]
      "Local [east_m,north_m,up_m] or global [lat_deg,lon_deg,alt_m] rows";
    input Real velocityEnu[capacity, 3]
      "World-frame velocity at each waypoint [m/s]";
    input Real yaw[capacity](each unit = "rad") "Heading at each waypoint";
    input Real nominalSpeed(unit = "m/s") "Nominal segment-average speed";
    input Real minSegmentDuration(unit = "s") "Lower duration bound";
  end WaypointPlanInput;

  connector TrajectoryReferenceOutput
    "Time-indexed differentially flat reference for a flight controller"
    output Boolean valid(start = false) "A plan has been accepted";
    output Boolean complete(start = false)
      "The accepted plan reached its final waypoint";
    output Integer sequence(start = -1) "Sequence number of the accepted plan";
    output Integer activeSegment(start = 1) "One-based segment index";
    output Real trajectoryTime(unit = "s", start = 0.0)
      "Time since the accepted plan began";
    output Real totalDuration(unit = "s", start = 0.0)
      "Duration of the accepted plan";
    output Real position[3](each unit = "m", each start = 0.0);
    output Real velocity[3](each unit = "m/s", each start = 0.0);
    output Real acceleration[3](each unit = "m/s2", each start = 0.0);
    output Real jerk[3](each unit = "m/s3", each start = 0.0);
    output Real snap[3](each unit = "m/s4", each start = 0.0);
    output Real yaw(unit = "rad", start = 0.0);
    output Real yawRate(unit = "rad/s", start = 0.0);
    output Real yawAcceleration(unit = "rad/s2", start = 0.0);
  end TrajectoryReferenceOutput;

  annotation(Documentation(info = "<html>
    <p>These connectors are message-shaped eFMU boundaries. Their waypoint
    tensors have fixed capacity while <code>waypointCount</code> identifies the
    populated prefix, which keeps generated storage bounded without exposing
    scalarized channels.</p>
  </html>"));
end Interfaces;
