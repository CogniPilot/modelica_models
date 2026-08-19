within Estimation.StrapdownINS;

function rangeToPlane
  "Distance from a co-located nadir camera to a plane along its -body-z ray"
  input Real positionWorldEnu_m[3];
  input Real quaternionWorldBody[4];
  input Real planeNormalWorldEnu[3] = {0.0, 0.0, 1.0};
  input Real planeOffset_m = 0.0 "d in normal' * position = d";
  output Real distance_m;
protected
  Real rotationWorldBody[3, 3];
  Real rayWorldEnu[3];
  Real denominator;
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(quaternionWorldBody);
  rayWorldEnu := rotationWorldBody * {0.0, 0.0, -1.0};
  denominator := planeNormalWorldEnu * rayWorldEnu;
  distance_m := (planeOffset_m
    - planeNormalWorldEnu * positionWorldEnu_m) / denominator;
end rangeToPlane;
