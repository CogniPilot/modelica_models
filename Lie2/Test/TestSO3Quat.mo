within Lie2.Test;

model TestSO3Quat
  import Lie2.SO3.EulerB321;
  import Lie2.SO3.Quat;
  constant Quat.Element g1 = Quat.one();
  constant Real[3, 3] M = Quat.toMatrix(g1);
  constant Quat.Element g2 = Quat.fromMatrix(EulerB321.toMatrix({1, 2, 3}));
  constant Quat.Element g3 = Quat.fromMatrix(EulerB321.toMatrix({0, 0, 0}));
  constant Quat.Element g4 = Quat.product(g1, g2);
  Quat.Element g5;
  EulerB321.Element e;
initial equation
  g5 = g3;
equation
  der(g5) = Quat.Dr(g5, {0, 0, 1});
  e = EulerB321.fromMatrix(Quat.toMatrix(g5));
end TestSO3Quat;