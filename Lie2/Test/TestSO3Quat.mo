within Lie2.Test;

model TestSO3Quat
  extends TestSO3Dcm(redeclare type G = SO3.Quat, test_matrix=false);
end TestSO3Quat;