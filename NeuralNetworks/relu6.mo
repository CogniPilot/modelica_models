within NeuralNetworks;

function relu6 "MobileNet bounded rectified activation"
  input Real x[:,:,:];
  output Real y[size(x,1),size(x,2),size(x,3)];
algorithm
  for row in 1:size(x,1) loop
    for column in 1:size(x,2) loop
      for channel in 1:size(x,3) loop
        y[row,column,channel] := if x[row,column,channel] < 0.0 then 0.0
          elseif x[row,column,channel] > 6.0 then 6.0
          else x[row,column,channel];
      end for;
    end for;
  end for;
end relu6;
