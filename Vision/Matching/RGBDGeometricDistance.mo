within Vision.Matching;
function RGBDGeometricDistance
  input Real first[3];
  input Real second[3];
  output Real distance;
protected
  Real delta;
algorithm
  distance := 0.0;
  for k in 1:3 loop
    delta := first[k]-second[k];
    distance := distance+delta*delta;
  end for;
end RGBDGeometricDistance;
