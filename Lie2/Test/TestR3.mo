within Lie2.Test;

model TestR3
  import Lie2.R3;
  R3.Group.Element g1 = {1, 2, 3};
  R3.Group.Element g2 = {4, 5, 6};
  R3.Group.Element g3, g4, g5;
  R3.Algebra.Element a1 = {1, 2, 3};
  R3.Algebra.Element a2 = {4, 5, 6};
  R3.Algebra.Element a3;

equation
  g3 = R3.Group.product(g1, g2);
  g4 = R3.Group.product(R3.Group.inv(g1),  g2);
  a3 = R3.Algebra.add(a1, a2);
  g5 = R3.Algebra.exp(a1);
end TestR3;
