within SLAM.Examples.Sensors;
// Editable IMU and GPS observation mathematics. Uniform draws are explicit
// replayable inputs. Camera noise and quantization run in the GPU depth shader.
model SensorObservations
  input Real accelTruth[3] = {0.0,0.0,9.81};
  input Real gyroTruth[3] = {0.0,0.0,0.0};
  input Real positionTruth[3] = {0.0,0.0,1.5};
  input Real accelDraws[3,2] = fill(0.5,3,2);
  input Real gyroDraws[3,2] = fill(0.5,3,2);
  input Real gpsDraws[3,2] = fill(0.5,3,2);
  input Real roofEnabled = 1.0;
  input Real roofMinimum[3] = {35.8,-5.2,0.0};
  input Real roofMaximum[3] = {48.2,5.2,3.8};
  output Real accel[3];
  output Real gyro[3];
  output Real gpsPosition[3];
  output Real gpsCovariance[3,3];
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
  parameter Real accelBias[3] = {0.015,-0.012,0.02};
  parameter Real gyroBias[3] = {0.0004,-0.0003,0.0006};
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
  for i in 1:3 loop
    accel[i] = accelTruth[i]+accelBias[i]+0.015*
      sqrt(-2.0*log(max(1e-9,accelDraws[i,1])))*cos(2.0*3.141592653589793*accelDraws[i,2]);
    gyro[i] = gyroTruth[i]+gyroBias[i]+0.0005*
      sqrt(-2.0*log(max(1e-9,gyroDraws[i,1])))*cos(2.0*3.141592653589793*gyroDraws[i,2]);
    gpsPosition[i] = positionTruth[i]+0.1*
      sqrt(-2.0*log(max(1e-9,gpsDraws[i,1])))*cos(2.0*3.141592653589793*gpsDraws[i,2]);
    for j in 1:3 loop
      gpsCovariance[i,j] = if i == j then 0.01 else 0.0;
    end for;
  end for;
end SensorObservations;
