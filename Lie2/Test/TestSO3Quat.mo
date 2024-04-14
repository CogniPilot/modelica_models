within Lie2.Test;

model TestSO3Quat
  extends TestSO3Dcm(redeclare type G = SO3.Quat);
end TestSO3Quat;