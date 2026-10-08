within SLAM.Mapping;
// Finite proper rotation gate. Invalid entries never enter matrix arithmetic.
model RGBDLandmarkRotationCheck
  constant Integer dimension = 3;
  constant Real tolerance = 1e-6;
  input Real rotation[dimension,dimension] = identity(dimension);
  output Real valid;
protected
  Real safeRotation[dimension,dimension];
  Real entryChecks[dimension,dimension];
  Real gram[dimension,dimension];
  Real gramChecks[dimension,dimension];
  Real determinant;
equation
  for i in 1:dimension loop
    for j in 1:dimension loop
      entryChecks[i,j] = if noEvent(abs(rotation[i,j]) <= 1.0+tolerance) then 0.0 else 1.0;
      safeRotation[i,j] = if noEvent(entryChecks[i,j] < 0.5) then rotation[i,j] else 0.0;
      gramChecks[i,j] = if noEvent(abs(gram[i,j]-(if i == j then 1.0 else 0.0)) <= tolerance) then 0.0 else 1.0;
    end for;
  end for;
  gram = transpose(safeRotation)*safeRotation;
  determinant = safeRotation[1,1]*(safeRotation[2,2]*safeRotation[3,3]-safeRotation[2,3]*safeRotation[3,2])
    -safeRotation[1,2]*(safeRotation[2,1]*safeRotation[3,3]-safeRotation[2,3]*safeRotation[3,1])
    +safeRotation[1,3]*(safeRotation[2,1]*safeRotation[3,2]-safeRotation[2,2]*safeRotation[3,1]);
  valid = if noEvent(sum(entryChecks)+sum(gramChecks) < 0.5 and abs(determinant-1.0) <= tolerance) then 1.0 else 0.0;
end RGBDLandmarkRotationCheck;
