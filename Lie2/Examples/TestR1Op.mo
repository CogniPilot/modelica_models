within Lie2.Examples;

model TestR1Op

  import Lie2.Op.R1Op;
  
  R1Op.AlgebraOp a1 = R1Op.AlgebraOp({1});
  R1Op.AlgebraOp a2 = R1Op.AlgebraOp({2});
  R1Op.AlgebraOp a3;
  
  R1Op.GroupOp g1 = R1Op.GroupOp({1});
  R1Op.GroupOp g2 = R1Op.GroupOp({2});
  R1Op.GroupOp g3;
  //R1Op.GroupOp g4;

equation

  a3 = a1 + a2;
  g3 = g1 * g2;
  //g4 = g1 * g1.inv();

end TestR1Op;