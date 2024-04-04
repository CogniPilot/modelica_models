within CogniPilot.Lie.SO3.Algebra;
encapsulated operator record Element
  import CogniPilot.Lie;

  extends Lie.Base.AlgebraElement;

  Real r[3];

  encapsulated operator '+'
    import CogniPilot.Lie.SO3.Algebra;

    function addition
      input Algebra.Element a,b;

      output Algebra.Element res;

    algorithm
      res := Algebra.addition(
        a,
        b);

    end addition;

  end '+';

  encapsulated operator '*'
    import CogniPilot.Lie.SO3.Algebra;

    function bracket
      input Algebra.Element a,b;

      output Algebra.Element res;

    algorithm
      res := Algebra.bracket(
        a,
        b);

    end bracket;

    function scalar_multiply
      import CogniPilot.Lie.SO3.Algebra;

      input Real a;

      input Algebra.Element b;

      output Algebra.Element res;

    algorithm
      res := Algebra.Element(
        r=a*b.r);

    end scalar_multiply;

  end '*';

  encapsulated operator function exp
    import CogniPilot.Lie.SO3;

    input SO3.Algebra a;

    input SO3.Base.Group group;

    output SO3.Base.GroupElement res;

  algorithm
    res := group.exp(
      a);

  end exp;

  encapsulated operator function ad=Lie.SO3.Algebra.adjoint(
    a=Lie.SO3.Algebra.Element(
      r));

  encapsulated operator function vee=Lie.SO3.Algebra.vee(
    a=Lie.SO3.Algebra.Element(
      r));

end Element;
