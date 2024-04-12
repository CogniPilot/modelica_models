within Lie2.Test;

model TestSO3Dcm
  import Lie2.SO3;
  SO3.Dcm.Element g1 = SO3.Dcm.one();
  SO3.Dcm.Element g2 = SO3.Dcm.one();
  SO3.Dcm.Element g3, g4;
initial equation
  g4 = SO3.Dcm.one();
equation
  g3 = SO3.Dcm.product(g1, g2);
  der(g4) = SO3.Dcm.Dr(g4, {1, 2, 3});
end TestSO3Dcm;
