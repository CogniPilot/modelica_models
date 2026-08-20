within Estimation.StrapdownINS;

function updateTerrainAltitude
  "Estimate locally horizontal terrain altitude from a downward slant range"
  input Boolean initializedPrevious;
  input Real altitudePrevious_m;
  input Real variancePrevious_m2;
  input Real positionWorldEnu_m[3];
  input Real quaternionWorldBody[4];
  input Real navigationCovarianceLocal[15, 15];
  input Avionics.OpticalFlowSample measurement;
  input Real dt(unit = "s");
  input Real initialAltitude_m = 0.0;
  input Real initialVariance_m2 = 4.0;
  input Real processNoise_m2_s = 1.0e-3;
  input Real minimumCosTilt = 0.5;
  input Real minimumQuality = 0.2;
  output Boolean initializedNext;
  output Real altitudeNext_m;
  output Real varianceNext_m2;
  output Boolean accepted;
protected
  Real rotationWorldBody[3, 3];
  Real cosTilt;
  Real terrainH[15];
  Real observedAltitude_m;
  Real predictedVariance_m2;
  Real observationVariance_m2;
  Real innovationVariance_m2;
  Real gain;
  Boolean usable;
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(quaternionWorldBody);
  cosTilt := rotationWorldBody[3, 3];
  terrainH := zeros(15);
  terrainH[1:3] := rotationWorldBody[3, :];
  // R_new = R*Exp(delta): d(R[3,3])/d(delta) = {-R[3,2],R[3,1],0}.
  terrainH[7:9] := measurement.groundDistance_m
    * {rotationWorldBody[3, 2], -rotationWorldBody[3, 1], 0.0};
  observedAltitude_m := positionWorldEnu_m[3]
    - measurement.groundDistance_m * cosTilt;
  predictedVariance_m2 := max(
    if initializedPrevious then variancePrevious_m2 else initialVariance_m2,
    1.0e-12) + max(processNoise_m2_s, 0.0) * max(dt, 0.0);
  observationVariance_m2 := cosTilt * cosTilt
      * measurement.groundDistanceVariance_m2
    + terrainH * navigationCovarianceLocal * terrainH;
  innovationVariance_m2 := predictedVariance_m2 + observationVariance_m2;
  usable := measurement.valid and measurement.fresh
    and measurement.quality >= minimumQuality
    and measurement.groundDistance_m > 0.05
    and measurement.groundDistanceVariance_m2 > 0.0
    and cosTilt >= minimumCosTilt
    and abs(observedAltitude_m) < 1.0e100
    and observationVariance_m2 > 0.0
    and innovationVariance_m2 > 1.0e-12;
  if usable then
    gain := predictedVariance_m2 / innovationVariance_m2;
    altitudeNext_m := (if initializedPrevious then altitudePrevious_m
      else initialAltitude_m) + gain * (observedAltitude_m
        - (if initializedPrevious then altitudePrevious_m else initialAltitude_m));
    varianceNext_m2 := max((1.0 - gain) * predictedVariance_m2, 1.0e-12);
    initializedNext := true;
    accepted := true;
  else
    altitudeNext_m := if initializedPrevious then altitudePrevious_m
      else initialAltitude_m;
    varianceNext_m2 := predictedVariance_m2;
    initializedNext := initializedPrevious;
    accepted := false;
  end if;
end updateTerrainAltitude;
