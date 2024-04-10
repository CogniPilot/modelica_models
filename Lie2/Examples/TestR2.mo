within Lie2.Examples;

model TestR2
  import Lie2.R2;
  
  R2.Group.Element g1 = {1, 2};
  R2.Group.Element g2 = {3, 4};
  R2.Group.Element g3, g4, g5;
  R2.Algebra.Element a1 = {1, 2};
  R2.Algebra.Element a2 = {3, 4};
  R2.Algebra.Element a3;

equation
  g3 = R2.Group.product(g1, g2);
  g4 = R2.Group.product(R2.Group.inv(g1),  g2);
  a3 = R2.Algebra.add(a1, a2);
  g5 = R2.Algebra.exp(a1);
end TestR2;