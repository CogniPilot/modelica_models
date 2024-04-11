within Lie2.Test;

model TestSO3Euler
  import Lie2.SO3.EulerB321;
  EulerB321.Element g1 = {1, 0, 0};
  EulerB321.Element g2 = {0, 1, 0};
  EulerB321.Element g3, g4;
  
equation
  g3 = EulerB321.product(g1, g2);
  g4 = EulerB321.identity();
end TestSO3Euler;
