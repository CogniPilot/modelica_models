within SLAM.Localization;
model RGBDLocalizationOrientation
  import SLAMRotationLog = SLAM.Inertial.SLAMRotationLog;

  extends SLAMRotationLog;
  output Real unitQuaternion[4];
equation
  unitQuaternion = if noEvent(valid > 0.5) then quaternion else {1.0,0.0,0.0,0.0};
end RGBDLocalizationOrientation;
