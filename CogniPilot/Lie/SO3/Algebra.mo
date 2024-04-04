within CogniPilot.Lie.SO3;
encapsulated operator record Algebra
  import CogniPilot.Lie;
  extends Lie.Algebra;
  encapsulated operator record Element
    import CogniPilot.Lie;
    extends Lie.Group.Element;
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
      input SO3.Group group;
      output SO3.Group.Element res;
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
  encapsulated operator function bracket
    import CogniPilot.Lie.SO3.Algebra;
    input Algebra.Element a,b;
    output Algebra.Element c;
  algorithm
    c := Algebra.Element(
      r={a.r[2]*b.r[3]-b.r[2]*a.r[3],a.r[3]*b.r[1]-b.r[3]*a.r[1],a.r[1]*b.r[2]-b.r[1]*a.r[2]});
  end bracket;
  encapsulated operator function addition
    import CogniPilot.Lie.SO3.Algebra;
    input Algebra.Element a,b;
    output Algebra.Element c;
  algorithm
    c := Algebra.Element(
      r=a.r+b.r);
  end addition;
  encapsulated operator function scalar_multiplication
    import CogniPilot.Lie.SO3.Algebra;
    input Real a;
    input Algebra.Element b;
    output Algebra.Element c;
  algorithm
    c := Algebra.Element(
      r=a*b.r);
  end scalar_multiplication;
  encapsulated operator function adjoint
    import CogniPilot.Lie.SO3.Algebra;
    input Real a;
    output Algebra.Element c;
  algorithm
    c := Algebra.to_matrix(
      a);
  end adjoint;
  encapsulated operator function wedge
    import CogniPilot.Lie.SO3.Algebra;
    input Real a[3];
    output Algebra.Element b;
  algorithm
    b := Algebra.Element(
      r=a);
  end wedge;
  encapsulated operator function vee
    import CogniPilot.Lie.SO3.Algebra;
    input Algebra.Element a;
    output Real b[3];
  algorithm
    b := a.r;
  end vee;
  encapsulated operator function to_matrix
    import CogniPilot.Lie.SO3.Algebra;
    input Algebra.Element a;
    output Real M[3,3];
  algorithm
    M[1,1] :=-a.r[3];
    M[2,1] := a.r[3];
    M[1,3] := a.r[2];
    M[3,1] :=-a.r[2];
    M[2,3] :=-a.r[1];
    M[3,2] := a.r[1];
  end to_matrix;
  encapsulated operator function from_matrix
    import CogniPilot.Lie.SO3.Algebra;
    input Real M[3,3];
    output Algebra.Element a;
  algorithm
    a := Algebra.Element(
      r={M[3,2],M[1,3],M[2,1]});
  end from_matrix;
end Algebra;
