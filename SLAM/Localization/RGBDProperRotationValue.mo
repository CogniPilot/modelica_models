within SLAM.Localization;
// Finite proper rotation gate shared by models and held-IMU functions.
function RGBDProperRotationValue
  input Real rotation[3,3] = identity(3);
  output Real valid;
protected
  constant Integer dimension = 3;
  constant Real tolerance = 1e-6;
  Real gram[dimension,dimension];
  Real checks[dimension,dimension];
  Real determinant;
algorithm
  gram := transpose(rotation)*rotation;
  determinant := rotation[1,1]*(rotation[2,2]*rotation[3,3]-rotation[2,3]*rotation[3,2])
    - rotation[1,2]*(rotation[2,1]*rotation[3,3]-rotation[2,3]*rotation[3,1])
    + rotation[1,3]*(rotation[2,1]*rotation[3,2]-rotation[2,2]*rotation[3,1]);
  for i in 1:dimension loop
    for j in 1:dimension loop
      checks[i,j] := if abs(rotation[i,j]) <= 1.0+tolerance
        and abs(gram[i,j]-(if i == j then 1.0 else 0.0)) <= tolerance then 0.0 else 1.0;
    end for;
  end for;
  valid := if sum(checks) < 0.5 and abs(determinant-1.0) <= tolerance then 1.0 else 0.0;
end RGBDProperRotationValue;
