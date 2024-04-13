within Lie2.Test;

model TestSO3EulerB321
  extends TestSO3Dcm(redeclare type G = SO3.EulerB321, test_matrix=false);
end TestSO3EulerB321;
