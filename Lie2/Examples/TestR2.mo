within Lie2.Examples;

model TestR2
  import Lie2.R2;
  
  R2.Group.Element g1(r={1, 2}), g2(r={3, 4});
  R2.Group.Element g3, g4, g5;
  R2.Algebra.Element a1(r={1, 2}), a2(r={3, 4}), a3;

equation
  g3 = R2.Group.product(g1, g2);
  g4 = R2.Group.product(R2.Group.inverse(g1),  g2);
  a3 = R2.Algebra.add(a1, a2);
  g5 = R2.Algebra.exp(a1);
end TestR2;