within Lie.Examples;

model TestR3

  Lie.R3.Algebra a(r={1, 2, 3});
  Lie.R3.Group g = Lie.R3.Algebra.exp(a);
  
end TestR3;