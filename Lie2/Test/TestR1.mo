within Lie2.Test;

model TestR1
  type Package = R1;
  type G = Package.Group;
  type A = Package.Algebra;
  constant Integer n = 1;
  G.Element g1 = {i for i in 1:Package.n};
  G.Element g2 = {n + i for i in 1:Package.n};
  G.Element g3, g4, g5;
  A.Element a1 = {i for i in 1:Package.n};
  A.Element a2 = {n + i for i in 1:Package.n};
  A.Element a3;

equation
  g3 = G.product(g1, g2);
  g4 = G.product(G.inv(g1), g2);
  a3 = A.add(a1, a2);
  g5 = A.exp(a1);
end TestR1;
