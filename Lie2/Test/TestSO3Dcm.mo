within Lie2.Test;

model TestSO3Dcm
  import Lie2.SO3;
  SO3.Dcm.Element g1 = SO3.Dcm.one();
  SO3.Dcm.Element g2 = SO3.Dcm.one();
  SO3.Dcm.Element g3;
  
equation
  g3 = SO3.Dcm.product(g1, g2);
end TestSO3Dcm;
