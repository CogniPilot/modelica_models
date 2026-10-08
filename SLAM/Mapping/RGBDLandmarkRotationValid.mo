within SLAM.Mapping;
// Pure ordered equivalent of the equation rotation gate above. Invalid entries
// are replaced before Gram/determinant arithmetic; its tolerance is unchanged.
function RGBDLandmarkRotationValid
  input Real rotation[3,3];
  output Real valid;
protected
  constant Real tolerance = 1e-6;
  Real safeRotation[3,3]; Real gram[3,3]; Real determinant;
  Real entryCount; Real gramCount;
algorithm
  safeRotation := zeros(3,3); entryCount := 0.0; gramCount := 0.0;
  for i in 1:3 loop
    for j in 1:3 loop
      if abs(rotation[i,j]) <= 1.0+tolerance then
        safeRotation[i,j] := rotation[i,j];
      else
        entryCount := entryCount+1.0;
      end if;
    end for;
  end for;
  gram := transpose(safeRotation)*safeRotation;
  for i in 1:3 loop
    for j in 1:3 loop
      if not (abs(gram[i,j]-(if i == j then 1.0 else 0.0)) <= tolerance) then
        gramCount := gramCount+1.0;
      end if;
    end for;
  end for;
  determinant := safeRotation[1,1]*(safeRotation[2,2]*safeRotation[3,3]-safeRotation[2,3]*safeRotation[3,2])
    -safeRotation[1,2]*(safeRotation[2,1]*safeRotation[3,3]-safeRotation[2,3]*safeRotation[3,1])
    +safeRotation[1,3]*(safeRotation[2,1]*safeRotation[3,2]-safeRotation[2,2]*safeRotation[3,1]);
  valid := if entryCount+gramCount < 0.5 and abs(determinant-1.0) <= tolerance then 1.0 else 0.0;
end RGBDLandmarkRotationValid;
