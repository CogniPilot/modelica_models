within Lie.SO3;
operator record Algebra
  extends Lie.Algebra;

  Real r[3];

  encapsulated operator '+'
    import Lie.SO3.Algebra;

    function add
      input Algebra a;

      input Algebra b;

      output Algebra res;

    algorithm
      res.r := a.r+b.r;

      annotation (
        Inline=true);

    end add;

  end '+';

  function leftJac
    input Algebra phi;

    output Real[3,3] res;

  protected
    Real x,c1,c2;

  algorithm
    x := sqrt(
      phi.r*phi.r);

    if x > 1e-3 then
      c1 :=(1-cos(
        x))/x^2;

      c2 :=(x-sin(
        x))/x^3;

    else
      c1 := 1/2-x^2/24+x^4/720;

      c2 := 1/6-x^2/120+x^4/50540;

    end if;

    res := identity(
      3)+c1*skew(
      phi.r)+c2*skew(
      phi.r)*skew(
      phi.r);

    annotation (
      Inline=true);

  end leftJac;

  function leftJacInv
    input Algebra phi;

    output Real[3,3] res;

  protected
    Real x,c1;

  algorithm
    x := sqrt(
      phi.r*phi.r);

    if x > 1e-3 then
      c1 :=(2-x/tan(
        x/2))/(2*x^2);

    else
      c1 := 1/12+x^2/720+x^4/30240;

    end if;

    res := identity(
      3)-1/2*skew(
      phi.r)+c1*skew(
      phi.r)*skew(
      phi.r);

    annotation (
      Inline=true);

  end leftJacInv;

  function rightJac
    input Algebra phi;

    output Real[3,3] res;

  protected
    Real x,c1,c2;

  algorithm
    x := sqrt(
      phi.r*phi.r);

    if x > 1e-3 then
      c1 :=(1-cos(
        x))/x^2;

      c2 :=(x-sin(
        x))/x^3;

    else
      c1 :=(1-cos(
        x))/x^2;

      c2 := 1/6-x^2/120+x^4/50540;

    end if;

    res := identity(
      3)-c1*skew(
      phi.r)+c2*skew(
      phi.r)*skew(
      phi.r);

    annotation (
      Inline=true);

  end rightJac;

  function rightJacInv
    input Algebra phi;

    output Real[3,3] res;

  protected
    Real x,c1;

  algorithm
    x := sqrt(
      phi.r*phi.r);

    if x > 1e-3 then
      c1 :=(2-x/tan(
        x/2))/(2*x^2);

    else
      c1 := 1/12+x^2/720+x^4/30240;

    end if;

    res := identity(
      3)+1/2*skew(
      phi.r)+c1*skew(
      phi.r)*skew(
      phi.r);

    annotation (
      Inline=true);

  end rightJacInv;

  function expDcm
    import Lie.SO3.Dcm;

    input Algebra phi;

    output Dcm res;

  protected
    Real x,c1,c2;

  algorithm
    x := sqrt(
      phi.r*phi.r);

    if x > 1e-3 then
      c1 := sin(
        x)/x;

      c2 :=(1-cos(
        x))/x^2;

    else
      c1 := 1-x^2/6+x^4/120;

      c2 := 1/2-x^2/24+x^4/720;

    end if;

    res.r := identity(
      3)+c1*skew(
      phi.r)+c2*skew(
      phi.r)*skew(
      phi.r);

    annotation (
      Inline=true);

  end expDcm;

  function fromMatrix
    "algebra element from matrix"
    input Real[3,3] a;

    output Algebra res;

  algorithm
    res.r := {a[3,2],a[1,3],a[2,1]};

    annotation (
      Inline=true);

  end fromMatrix;

  function toMatrix
    "matrix to algebra element"
    input Algebra a;

    output Real[3,3] res;

  algorithm
    res := skew(
      a.r);

    annotation (
      Inline=true);

  end toMatrix;

end Algebra;
