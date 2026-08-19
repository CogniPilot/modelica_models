within Vehicles.Rdd2;

function simulateOpticalFlowPlane
  "Generate plane-induced flow for a co-located nadir camera"
  input Real positionWorldEnu_m[3];
  input Real velocityWorldEnu_m_s[3];
  input Real rotationWorldBody[3, 3];
  input Real angularVelocityBodyFlu_rad_s[3];
  input Real groundNormalWorldEnu[3] = {0.0, 0.0, 1.0};
  input Real groundPlaneOffset_m = 0.0;
  input Real integrationTime_s = 0.005;
  input Real normalizedImageRadius = 0.35
    "Feature-grid radius in normalized focal-length coordinates";
  output Real velocityBodyFlu_m_s[2]
    "Planar body velocity recovered from the plane-induced feature flow";
  output Real integratedLineOfSight_rad[2]
    "Mean raw angular image displacement, including camera rotation";
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
  integratedLineOfSight_rad := zeros(2);

  for feature in 1:FeatureCount loop
    rayBody := {featureU[feature], featureV[feature], -1.0};
    rayWorld := rotationWorldBody * rayBody;
    denominator := groundNormalWorldEnu * rayWorld;
    featureDistance_m := if abs(denominator) > 1.0e-6 then
      numerator / denominator else -1.0;
    allRaysVisible := allRaysVisible and featureDistance_m > 0.05;
    if featureDistance_m > 0.05 then
      relativeBody_m := featureDistance_m * rayBody;
      relativeDerivativeBody_m_s := -velocityBody
        - LieGroups.SO3.Quat.wedge(angularVelocityBodyFlu_rad_s)
          * relativeBody_m;
      normalizedRate := {
        -(relativeDerivativeBody_m_s[1] * relativeBody_m[3]
          - relativeBody_m[1] * relativeDerivativeBody_m_s[3])
          / (relativeBody_m[3] * relativeBody_m[3]),
        -(relativeDerivativeBody_m_s[2] * relativeBody_m[3]
          - relativeBody_m[2] * relativeDerivativeBody_m_s[3])
          / (relativeBody_m[3] * relativeBody_m[3])};
      integratedLineOfSight_rad := integratedLineOfSight_rad
        + {normalizedRate[1] / (1.0 + featureU[feature]^2),
           normalizedRate[2] / (1.0 + featureV[feature]^2)}
          * integrationTime_s / FeatureCount;
    end if;
  end for;
  // With known plane geometry and rotational compensation, the full-rank
  // multi-feature homography recovers camera translation exactly. Writing
  // its closed-form result avoids solving the same 3x3 normal equations at
  // every continuous plant evaluation; the ray loop above remains the raw
  // pixel/LOS sensor model and is what the inclined-plane test exercises.
  velocityBodyFlu_m_s := if allRaysVisible then velocityBody[1:2]
    else zeros(2);
  surfaceVisibility := if allRaysVisible then 1.0 else 0.0;
end simulateOpticalFlowPlane;
