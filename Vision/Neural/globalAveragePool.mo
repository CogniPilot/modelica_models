within Vision.Neural;

function globalAveragePool "Average each channel over all image coordinates"
  input Real x[:,:,:];
  output Real y[size(x,3)];
algorithm
  y := zeros(size(x,3));
  for channel in 1:size(x,3) loop
    for row in 1:size(x,1) loop
      for column in 1:size(x,2) loop
        y[channel] := y[channel] + x[row,column,channel];
      end for;
    end for;
    y[channel] := y[channel]/(size(x,1)*size(x,2));
  end for;
end globalAveragePool;
