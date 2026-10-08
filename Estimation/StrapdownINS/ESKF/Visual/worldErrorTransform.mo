within Estimation.StrapdownINS.ESKF.Visual;

function worldErrorTransform
  "Map ordinary right ESKF errors to world position/velocity, right attitude, accel/gyro biases"
  input Real rotationWorldBody[3, 3];
  output Real transform[15, 15];
algorithm
  transform := zeros(15, 15);
  transform[1:3, 1:3] := rotationWorldBody;
  transform[4:6, 4:6] := rotationWorldBody;
  transform[7:9, 7:9] := 1.0 * identity(3);
  transform[10:12, 13:15] := 1.0 * identity(3);
  transform[13:15, 10:12] := 1.0 * identity(3);
end worldErrorTransform;
