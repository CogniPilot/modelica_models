within NeuralNetworks;

function softmax "Numerically stable normalized exponential"
  input Real logits[:];
  output Real probability[size(logits,1)];
protected
  Real maximum;
  Real denominator;
algorithm
  assert(size(logits,1) > 0,"softmax requires at least one class");
  maximum := max(logits);
  denominator := 0.0;
  for classIndex in 1:size(logits,1) loop
    probability[classIndex] := exp(logits[classIndex]-maximum);
    denominator := denominator + probability[classIndex];
  end for;
  probability := probability/denominator;
end softmax;
