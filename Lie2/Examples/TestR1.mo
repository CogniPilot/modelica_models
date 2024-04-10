within Lie2.Examples;

model TestR1
  import Lie2.R1;
  
  R1.Group.Element g1(r={1}), g2(r={2});
  R1.Group.Element g3, g4, g5;
  R1.Algebra.Element a1(r={1}), a2(r={2}), a3;

equation
  g3 = R1.Group.product(g1, g2);
  g4 = R1.Group.product(Rn.Group.inv(g1), g2);
  a3 = R1.Algebra.add(a1, a2);
  g5 = R1.Algebra.exp(a1);
end TestR1;
