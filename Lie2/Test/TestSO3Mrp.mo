within Lie2.Test;

model TestSO3Mrp
  import Lie2.SO3;
  SO3.Mrp.Element g1 = {1, 0, 0};
  SO3.Mrp.Element g2 = {0, 1, 0};
  SO3.Mrp.Element g3, g4;
  
equation
  g3 = SO3.Mrp.product(g1, g2);
  g4 = SO3.Mrp.identity();
end TestSO3Mrp;
