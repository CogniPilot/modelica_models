within Lie2.SO2;

operator record OpAlgebraElement
  Real r[Rn.Group.n_param];

  encapsulated operator '+'
    import Lie2.Rn.Algebra;
    function product = Algebra.add;
  end '+';

  function exp = Algebra.exp(a = AlgebraElement(r = r));
  Real r[Rn.Group.n_param];
  Real r[Rn.Group.n_param];
  Real r[Rn.Group.n_param];
end OpAlgebraElement;