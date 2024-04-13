within Lie2.Test;

model TestSO3Mrp
  extends TestSO3Dcm(redeclare type G = SO3.Mrp, test_matrix=false);
end TestSO3Mrp;
