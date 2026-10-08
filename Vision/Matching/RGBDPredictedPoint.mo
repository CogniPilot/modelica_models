within Vision.Matching;
function RGBDPredictedPoint
  input Real point[3];
  input Real rotation[3,3];
  input Real translation[3];
  input Boolean enabled;
  output Real predicted[3];
algorithm
  predicted := zeros(3);
  for a in 1:3 loop
    predicted[a] := if enabled then translation[a] else 0.0;
    for b in 1:3 loop
      predicted[a] := predicted[a]+(if enabled then rotation[a,b]*point[b] else 0.0);
    end for;
  end for;
end RGBDPredictedPoint;
