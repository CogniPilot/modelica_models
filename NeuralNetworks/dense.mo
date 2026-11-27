within NeuralNetworks;

function dense "Affine fully connected layer"
  input Real x[:];
  input Real weight[:,size(x,1)] "Output channel by input channel";
  input Real bias[size(weight,1)] = zeros(size(weight,1));
  output Real y[size(weight,1)];
algorithm
  y := bias + weight*x;
end dense;
