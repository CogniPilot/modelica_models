within Vision.Matching;
function RGBDDescriptorDistance
  input Real first[:];
  input Real second[size(first,1)];
  output Real distance;
protected
  Real delta;
algorithm
  distance := 0.0;
  for k in 1:size(first,1) loop
    delta := first[k]-second[k];
    distance := distance+delta*delta;
  end for;
end RGBDDescriptorDistance;
