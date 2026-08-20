within Estimation.StrapdownINS;

function updateBarometerBias
  "Update a scalar pressure-altitude bias estimate against navigation altitude"
  input Boolean initializedPrevious;
  input Real biasPrevious_m;
  input Real variancePrevious_m2;
  input Real positionWorldEnu_m[3];
  input Real quaternionWorldBody[4];
  input Real navigationCovarianceLocal[15, 15];
  input Avionics.BarometerSample measurement;
  input Real dt(unit = "s");
  input Real initialBias_m = 0.0;
  input Real initialVariance_m2 = 4.0;
  input Real processNoise_m2_s = 1.0e-4;
  output Boolean initializedNext;
  output Real biasNext_m;
  output Real varianceNext_m2;
  output Boolean accepted;
protected
  Real rotationWorldBody[3, 3];
  Real altitudeH[15];
  Real predictedVariance_m2;
  Real observationVariance_m2;
  Real innovationVariance_m2;
  Real gain;
  Real innovation_m;
  Boolean usable;
algorithm
  rotationWorldBody := LieGroups.SO3.Quat.to_DCM(quaternionWorldBody);
  altitudeH := zeros(15);
  altitudeH[1:3] := rotationWorldBody[3, :];
  predictedVariance_m2 := max(
    if initializedPrevious then variancePrevious_m2 else initialVariance_m2,
    1.0e-12) + max(processNoise_m2_s, 0.0) * max(dt, 0.0);
  observationVariance_m2 := measurement.variance_m2
    + altitudeH * navigationCovarianceLocal * altitudeH;
  innovationVariance_m2 := predictedVariance_m2 + observationVariance_m2;
  innovation_m := measurement.altitudeWorldEnu_m - positionWorldEnu_m[3]
    - (if initializedPrevious then biasPrevious_m else initialBias_m);
  usable := measurement.valid and measurement.fresh
    and abs(measurement.timestamp_s) < 1.0e100
    and abs(measurement.altitudeWorldEnu_m) < 1.0e100
    and measurement.variance_m2 > 0.0
    and observationVariance_m2 > 0.0
    and innovationVariance_m2 > 1.0e-12;
  if usable then
    gain := predictedVariance_m2 / innovationVariance_m2;
    biasNext_m := (if initializedPrevious then biasPrevious_m
      else initialBias_m) + gain * innovation_m;
    varianceNext_m2 := max((1.0 - gain) * predictedVariance_m2, 1.0e-12);
    initializedNext := true;
    accepted := true;
  else
    biasNext_m := if initializedPrevious then biasPrevious_m else initialBias_m;
    varianceNext_m2 := predictedVariance_m2;
    initializedNext := initializedPrevious;
    accepted := false;
  end if;
end updateBarometerBias;
