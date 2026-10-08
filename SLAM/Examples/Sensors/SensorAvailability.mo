within SLAM.Examples.Sensors;
// Evaluate this pure gate before consuming any independent GPS uniform draws.
// An inclusive demonstration roof volume, not an RF/satellite visibility model.
model SensorAvailability
  input Real positionTruth[3] = {0.0,0.0,1.5};
  input Real roofEnabled = 1.0;
  input Real roofMinimum[3] = {35.8,-5.2,0.0};
  input Real roofMaximum[3] = {48.2,5.2,3.8};
  // Fixed slot capacity keeps the editable source structurally compact.
  // Slot one retains the original roofMinimum/roofMaximum input interface.
  constant Integer maximumRoofs = 3;
  input Real roofCount = 1.0;
  input Real roofMinima[maximumRoofs,3] = fill(0.0,maximumRoofs,3);
  input Real roofMaxima[maximumRoofs,3] = fill(0.0,maximumRoofs,3);
  output Real roofConfigurationValid;
  output Real gpsAvailable;
protected
  Real roofContains[maximumRoofs];
  Real roofBoundsValid[maximumRoofs];
equation
  for roof in 1:maximumRoofs loop
    roofBoundsValid[roof] = if noEvent(roof > roofCount or
      (if roof == 1 then
        roofMinimum[1] <= roofMaximum[1] and roofMinimum[2] <= roofMaximum[2] and roofMinimum[3] <= roofMaximum[3]
      else roofMinima[roof,1] <= roofMaxima[roof,1] and roofMinima[roof,2] <= roofMaxima[roof,2] and roofMinima[roof,3] <= roofMaxima[roof,3])) then 1.0 else 0.0;
    roofContains[roof] = if noEvent(roof <= roofCount and
      (if roof == 1 then
        positionTruth[1] >= roofMinimum[1] and positionTruth[1] <= roofMaximum[1] and
        positionTruth[2] >= roofMinimum[2] and positionTruth[2] <= roofMaximum[2] and
        positionTruth[3] >= roofMinimum[3] and positionTruth[3] <= roofMaximum[3]
      else positionTruth[1] >= roofMinima[roof,1] and positionTruth[1] <= roofMaxima[roof,1] and
        positionTruth[2] >= roofMinima[roof,2] and positionTruth[2] <= roofMaxima[roof,2] and
        positionTruth[3] >= roofMinima[roof,3] and positionTruth[3] <= roofMaxima[roof,3])) then 1.0 else 0.0;
  end for;
  roofConfigurationValid = if noEvent(roofCount >= 0.0 and roofCount <= maximumRoofs and
    abs(roofCount-floor(roofCount)) <= 0.0 and sum(roofBoundsValid) >= maximumRoofs) then 1.0 else 0.0;
  gpsAvailable = if noEvent(roofConfigurationValid < 0.5 or
    (roofEnabled > 0.5 and sum(roofContains) > 0.0)) then 0.0 else 1.0;
end SensorAvailability;
