within Vehicles.Cubs2;

// SPDX-License-Identifier: Apache-2.0

package Interfaces "Typed tensor and message boundaries for CUBS2"
  record Odometry
    Real timestamp_ns;
    Real position_m[3];
    Real quaternion[4] "body-to-world quaternion {w,x,y,z}";
    Real velocity_m_s[3];
    Real angularVelocity_rad_s[3];
    Real flags;
    Real status;
    Real estimatorType;
    Real resetCounter;
    Real qualityPct;
    Real timeStatus;
  end Odometry;

  record AutopilotDebug
    Real currentWaypoint;
    Real desiredSpeed_m_s;
    Real rollCommand_rad;
    Real courseError_rad;
  end AutopilotDebug;
end Interfaces;
