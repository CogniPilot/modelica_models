within SLAM.Localization;
model RGBDProperRotation
  import RGBDProperRotationValue = SLAM.Localization.RGBDProperRotationValue;

  constant Integer dimension = 3;
  input Real rotation[dimension,dimension] = identity(dimension);
  constant Real tolerance = 1e-6;
  output Real valid;
equation
  valid = RGBDProperRotationValue(rotation);
end RGBDProperRotation;
