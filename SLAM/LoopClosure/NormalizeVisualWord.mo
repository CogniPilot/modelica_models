within SLAM.LoopClosure;
// Appearance retrieval only; full descriptor matching and geometric registration
// must verify each loop proposal.
function NormalizeVisualWord
  input Real descriptor[descriptorSize];
  input Real enabled;
  output Real normalized[descriptorSize];
  output Boolean valid;
protected
  constant Integer descriptorSize = 49;
  Real mean;
  Real energy;
  Real value;
  Real scale;
algorithm
  normalized := zeros(descriptorSize);
  mean := 0.0;
  energy := 0.0;
  value := 0.0;
  scale := 1.0;
  valid := enabled >= 1.0 and enabled <= 1.0;
  for k in 1:descriptorSize loop
    valid := valid and abs(descriptor[k]) <= 1e3;
    value := if abs(descriptor[k]) <= 1e3 then descriptor[k] else 0.0;
    normalized[k] := value;
    mean := mean+value/descriptorSize;
  end for;
  for k in 1:descriptorSize loop
    normalized[k] := normalized[k]-mean;
    energy := energy+normalized[k]*normalized[k];
  end for;
  valid := valid and energy > 1e-12 and energy <= 1e9;
  scale := if valid then sqrt(max(energy,1e-12)) else 1.0;
  for k in 1:descriptorSize loop
    normalized[k] := if valid then normalized[k]/scale else 0.0;
  end for;
end NormalizeVisualWord;
