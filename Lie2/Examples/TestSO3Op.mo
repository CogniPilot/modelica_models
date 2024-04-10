within Lie2.Examples;

model TestSO3Op

  import Lie2.Op.SO3Op;
  
  SO3Op.AlgebraOp a1 = SO3Op.AlgebraOp({1, 2, 3});
  SO3Op.AlgebraOp a2 = SO3Op.AlgebraOp({4, 5, 6});
  SO3Op.AlgebraOp a3;
  
  SO3Op.Dcm.GroupOp g1 = SO3Op.Dcm.GroupOp(identity(3));
  SO3Op.Dcm.GroupOp g2 = SO3Op.Dcm.GroupOp(identity(3));
  SO3Op.Dcm.GroupOp g3, g4;

equation

  a3 = a1 + a2;
  g3 = g1 * g2;
  g4 = g1 * g1.inv();

end TestSO3Op;