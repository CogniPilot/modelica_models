within Lie.Examples;

model TestR1

  //Lie.R1.Algebra a(r={1}), b(r={2}), c;
  //Lie.R1.Group g = Lie.R1.Algebra.exp(a);
  Lie.R1.Group g1(r={1}), g2(r={2}), g3;

equation
  //c = a + b;
  g3 = g1*g2;
  
end TestR1;