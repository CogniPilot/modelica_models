within Vision.Neural;

function scaleChannels "Apply one squeeze/excitation gate per image channel"
  input Real x[:,:,:];
  input Real scale[size(x,3)];
  output Real y[size(x,1),size(x,2),size(x,3)];
algorithm
  for row in 1:size(x,1) loop
    for column in 1:size(x,2) loop
      for channel in 1:size(x,3) loop
        y[row,column,channel] := x[row,column,channel]*scale[channel];
      end for;
    end for;
  end for;
end scaleChannels;
