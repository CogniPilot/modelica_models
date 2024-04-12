within Lie2.Test;

model TestSO3Quat
  import Lie2.SO3.EulerB321;
  import Lie2.SO3.Quat;
  Quat.Element g1 = Quat.one();
  Quat.Element g2 = Quat.one();
  Quat.Element g3, g4, g5, g6;
  Real[3, 3] M;
  EulerB321.Element g7;
initial equation
  g6 = Quat.fromMatrix(EulerB321.toMatrix({0, 0, 0}));
equation
  g3 = Quat.product(g1, g2);
  g4 = Quat.one();
  M = Quat.toMatrix(g3);
  g5 = Quat.fromMatrix(M);
  der(g6) = Quat.Dr(g6, {0, 0, 1});
  g7 = EulerB321.fromMatrix(Quat.toMatrix(g6));
end TestSO3Quat;