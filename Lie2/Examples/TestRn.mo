within Lie2.Examples;

model TestRn
  import Lie2.Rn;
  
  Rn.Group.Element g1(r={1}), g2(r={2});
  Rn.Group.Element g3, g4, g5;
  Rn.Algebra.Element a1(r={1}), a2(r={2}), a3;

equation
  g3 = Rn.Group.product(g1, g2);
  g4 = Rn.Group.product(Rn.Group.inverse(g1), g2);
  a3 = Rn.Algebra.add(a1, a2);
  g5 = Rn.Algebra.exp(a1);
end TestRn;
