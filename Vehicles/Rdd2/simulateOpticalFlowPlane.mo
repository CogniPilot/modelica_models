within Vehicles.Rdd2;

function simulateOpticalFlowPlane
  "Generate raw plane-induced flow for a co-located nadir camera"
  input Real positionWorldEnu_m[3];
  input Real velocityWorldEnu_m_s[3];
  input Real rotationWorldBody[3, 3];
  input Real angularVelocityBodyFlu_rad_s[3];
  input Real groundNormalWorldEnu[3] = {0.0, 0.0, 1.0};
  input Real groundPlaneOffset_m = 0.0;
  input Real integrationTime_s = 0.005;
  input Real normalizedImageRadius = 0.35
    "Feature-grid radius in normalized focal-length coordinates";
  output Real integratedLineOfSight_rad[2]
    "Mean raw right-handed scene rotation about body FLU x/y";
  output Real groundDistance_m
    "Distance along the central -body-z camera ray";
  output Real surfaceVisibility
    "One when every feature ray intersects the plane in front of the camera";
protected
  constant Integer FeatureCount = 9;
  Real featureU[FeatureCount];
  Real featureV[FeatureCount];
  Real rayBody[3];
  Real rayWorld[3];
  Real denominator;
  Real numerator;
  Real featureDistance_m;
  Real relativeBody_m[3];
  Real relativeDerivativeBody_m_s[3];
  Real normalizedRate[2];
  Real translationFlow_rad[2];
  Real flatFieldScale;
  Real velocityBody[3];
  Boolean allRaysVisible;
algorithm
  featureU := {-normalizedImageRadius, 0.0, normalizedImageRadius,
    -normalizedImageRadius, 0.0, normalizedImageRadius,
    -normalizedImageRadius, 0.0, normalizedImageRadius};
  featureV := {-normalizedImageRadius, -normalizedImageRadius,
    -normalizedImageRadius, 0.0, 0.0, 0.0,
    normalizedImageRadius, normalizedImageRadius, normalizedImageRadius};
  velocityBody := transpose(rotationWorldBody) * velocityWorldEnu_m_s;
  numerator := groundPlaneOffset_m
    - groundNormalWorldEnu * positionWorldEnu_m;
  denominator := groundNormalWorldEnu
    * (rotationWorldBody * {0.0, 0.0, -1.0});
  allRaysVisible := false;
  if abs(denominator) > 1.0e-6 then
    allRaysVisible := numerator / denominator > 0.05;
  end if;
  groundDistance_m := if allRaysVisible then numerator / denominator else 0.0;
  translationFlow_rad := zeros(2);

  for feature in 1:FeatureCount loop
    rayBody := {featureU[feature], featureV[feature], -1.0};
    rayWorld := rotationWorldBody * rayBody;
    denominator := groundNormalWorldEnu * rayWorld;
    featureDistance_m := if abs(denominator) > 1.0e-6 then
      numerator / denominator else -1.0;
    allRaysVisible := allRaysVisible and featureDistance_m > 0.05;
    if featureDistance_m > 0.05 then
      relativeBody_m := featureDistance_m * rayBody;
      relativeDerivativeBody_m_s := -velocityBody;
      normalizedRate := {
        -(relativeDerivativeBody_m_s[1] * relativeBody_m[3]
          - relativeBody_m[1] * relativeDerivativeBody_m_s[3])
          / (relativeBody_m[3] * relativeBody_m[3]),
        -(relativeDerivativeBody_m_s[2] * relativeBody_m[3]
          - relativeBody_m[2] * relativeDerivativeBody_m_s[3])
          / (relativeBody_m[3] * relativeBody_m[3])};
      translationFlow_rad := translationFlow_rad
        + {normalizedRate[2] / (1.0 + featureV[feature]^2),
           -normalizedRate[1] / (1.0 + featureU[feature]^2)}
          * integrationTime_s / FeatureCount;
    end if;
  end for;
  // A real flow unit calibrates its field aggregate so flat-ground lateral
  // translation obeys flow = velocity / center range.  For this symmetric
  // three-by-three grid the raw angular average has the exact scale below.
  // Apply that calibration only to translation: co-timed gyro rotation must
  // remain one-for-one, while feature-dependent depths still preserve the
  // observable effect of a plane inclined relative to the camera.
  flatFieldScale := (1.0 + 2.0
    / (1.0 + normalizedImageRadius * normalizedImageRadius)) / 3.0;
  integratedLineOfSight_rad := if allRaysVisible then
    translationFlow_rad / flatFieldScale
      - angularVelocityBodyFlu_rad_s[1:2] * integrationTime_s
    else zeros(2);
  surfaceVisibility := if allRaysVisible then 1.0 else 0.0;
end simulateOpticalFlowPlane;
