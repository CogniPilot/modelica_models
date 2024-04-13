within Lie2.Test;

model TestR1
  type Package = R1;
  type G = Package.Group;
  type A = Package.Algebra;
  constant Integer n = Package.n;
  G.Element g1 = {i for i in 1:n};
  G.Element g2 = {n + i for i in 1:n};
  G.Element g3, g4;
  A.Element a1 = {i for i in 1:n};
  A.Element a2 = {n + i for i in 1:n};
  A.Element a3;
equation
  g3 = G.product(g1, g2);
  g4 = G.product(G.inv(g1), g2);
  a3 = A.add(a1, a2);
  assert(A.equal(G.log(G.exp(a1)), a1), "exp/log failed");
  assert(A.equal(A.fromMatrix(A.toMatrix(a1)), a1), "algebra matrix failed");
  assert(G.equal(G.fromMatrix(G.toMatrix(g3)), g3), "group matrix failed");
end TestR1;
