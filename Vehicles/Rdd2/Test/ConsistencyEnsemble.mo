within Vehicles.Rdd2.Test;

package ConsistencyEnsemble
  "Seed-varied mission wrappers for the NEES/NIS consistency study"

  model GpsSeed1
    extends GlobalWaypointMission(sensorNoiseSeed = 20260818.0);
  end GpsSeed1;

  model GpsSeed2
    extends GlobalWaypointMission(sensorNoiseSeed = 101.0);
  end GpsSeed2;

  model GpsSeed3
    extends GlobalWaypointMission(sensorNoiseSeed = 202.0);
  end GpsSeed3;

  model GpsSeed4
    extends GlobalWaypointMission(sensorNoiseSeed = 303.0);
  end GpsSeed4;

  model GpsSeed5
    extends GlobalWaypointMission(sensorNoiseSeed = 404.0);
  end GpsSeed5;

  model FlowSeed1
    extends OpticalFlowGpsDeniedMission(sensorNoiseSeed = 20260818.0);
  end FlowSeed1;

  model FlowSeed2
    extends OpticalFlowGpsDeniedMission(sensorNoiseSeed = 101.0);
  end FlowSeed2;

  model FlowSeed3
    extends OpticalFlowGpsDeniedMission(sensorNoiseSeed = 202.0);
  end FlowSeed3;

  model FlowSeed4
    extends OpticalFlowGpsDeniedMission(sensorNoiseSeed = 303.0);
  end FlowSeed4;

  model FlowSeed5
    extends OpticalFlowGpsDeniedMission(sensorNoiseSeed = 404.0);
  end FlowSeed5;

  annotation(Documentation(info="<html>
    <p>Ten thin wrappers over the two qualification missions, varying only
    the deterministic sensor-noise seed, so a five-member ensemble of each
    mission supports ANEES/ANIS consistency evaluation. The estimator
    consistency channels (full error covariance and bias estimates) are
    exported by the ESKF estimator's instrumentation outputs.</p>
  </html>"));
end ConsistencyEnsemble;
