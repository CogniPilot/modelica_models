within NeuralNetworks;

function hardSigmoid "Piecewise-linear MobileNet gate"
  input Real x[:];
  output Real y[size(x,1)];
algorithm
  for channel in 1:size(x,1) loop
    y[channel] := if x[channel] <= -3.0 then 0.0
      elseif x[channel] >= 3.0 then 1.0
      else (x[channel]+3.0)/6.0;
  end for;
end hardSigmoid;
