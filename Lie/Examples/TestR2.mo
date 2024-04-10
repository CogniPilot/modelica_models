within Lie.Examples;

model TestR2

  Lie.R2.Algebra a(r={1, 2});
  Lie.R2.Group g = Lie.R2.Algebra.exp(a);
  
end TestR2;