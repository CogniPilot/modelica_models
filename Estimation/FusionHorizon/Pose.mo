within Estimation.FusionHorizon;

record Pose "Navigation pose carried by the horizon, with no filter content"
  Real positionWorldEnu_m[3](each unit = "m");
  Real velocityWorldEnu_m_s[3](each unit = "m/s");
  Real quaternionWorldBody[4](each unit = "1")
    "Scalar-first Hamilton quaternion {w,x,y,z}, body FLU to world ENU";
end Pose;
