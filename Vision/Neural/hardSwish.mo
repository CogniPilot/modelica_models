within Vision.Neural;

function hardSwish "Piecewise-linear MobileNetV3 activation"
  input Real x[:,:,:];
  output Real y[size(x,1),size(x,2),size(x,3)];
algorithm
  for row in 1:size(x,1) loop
    for column in 1:size(x,2) loop
      for channel in 1:size(x,3) loop
        y[row,column,channel] := if x[row,column,channel] <= -3.0 then 0.0
          elseif x[row,column,channel] >= 3.0 then x[row,column,channel]
          else x[row,column,channel]*(x[row,column,channel]+3.0)/6.0;
      end for;
    end for;
  end for;
end hardSwish;
