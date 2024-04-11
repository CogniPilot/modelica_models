within Lie2.Test;

model TestSO3Algebra
  import Lie2.SO3;
  SO3.Algebra.Element a1 = {1, 2, 3};
  SO3.Algebra.Element a2 = {4, 5, 6};
  SO3.Algebra.Element a3;

equation
  a3 = SO3.Algebra.bracket(a1, a2);
end TestSO3Algebra;
