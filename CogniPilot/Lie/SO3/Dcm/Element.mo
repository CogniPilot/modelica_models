within CogniPilot.Lie.SO3.Dcm;
encapsulated operator record Element
  import CogniPilot.Lie;

  extends Lie.SO3.Base.GroupElement;

  Real r[3,3];

  encapsulated operator '*'
    import CogniPilot.Lie.SO3.Dcm;

    function product
      input Dcm.Element a,b;

      output Dcm.Element res;

    algorithm
      res := Dcm.Group.product(
        a,
        b);

    end product;

  end '*';

  encapsulated operator '=='
    import CogniPilot.Lie.SO3.Dcm;

    import CogniPilot.Math;

    function equal
      input Dcm.Element a,b;

      output Boolean res;

    algorithm
      res := Dcm.Group.equal(
        a,
        b,
        Math.eps);

    end equal;

  end '==';

  encapsulated operator function Ad=Lie.SO3.Dcm.Group.adjoint(
    a=Lie.SO3.Dcm.Element(
      r));

  encapsulated operator function log=Lie.SO3.Dcm.Group.log(
    a=Lie.SO3.Dcm.Element(
      r));

  encapsulated operator function to_matrix=Lie.SO3.Dcm.Group.to_matrix(
    a=Lie.SO3.Dcm.Element(
      r));

  encapsulated operator function inverse=Lie.SO3.Dcm.Group.inverse(
    a=Lie.SO3.Dcm.Element(
      r));

end Element;
