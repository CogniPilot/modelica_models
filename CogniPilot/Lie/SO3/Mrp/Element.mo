within CogniPilot.Lie.SO3.Mrp;
encapsulated operator record Element
  import CogniPilot.Lie;

  extends Lie.Base.GroupElement;

  Real r[3];

  encapsulated operator '*'
    import CogniPilot.Lie.SO3.Mrp;

    function product
      input Mrp.Element a,b;

      output Mrp.Element res;

    algorithm
      res := Mrp.Group.product(
        a,
        b);

    end product;

  end '*';

  encapsulated operator '=='
    import CogniPilot.Lie.SO3.Mrp;

    import CogniPilot.Math;

    function equal
      input Mrp.Element a,b;

      output Boolean res;

    algorithm
      res := Mrp.Group.equal(
        a,
        b,
        Math.eps);

    end equal;

  end '==';

  encapsulated operator function Ad=Lie.SO3.Mrp.Group.adjoint(
    a=Lie.SO3.Mrp.Element(
      r));

  encapsulated operator function log=Lie.SO3.Mrp.Group.log(
    a=Lie.SO3.Mrp.Element(
      r));

  encapsulated operator function to_matrix=Lie.SO3.Mrp.Group.to_matrix(
    a=Lie.SO3.Mrp.Element(
      r));

  encapsulated operator function inverse=Lie.SO3.Mrp.Group.inverse(
    a=Lie.SO3.Mrp.Element(
      r));

end Element;
