within CogniPilot.Lie.SO3.Mrp;
operator record Group
  extends SO3.Base.Group;

  encapsulated operator function elem
    import CogniPilot.Lie.SO3.Mrp;

    input Real[3] r;

    output Mrp.Element res;

  algorithm
    res := Mrp.Element(
      r=r);

  end elem;

  encapsulated operator function product
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a,b;

    output Mrp.Element c;

  protected
    Real na_sq,nb_sq,den,res[3];

  algorithm
    na_sq := a.r*a.r;

    nb_sq := b.r*b.r;

    den := 1+na_sq*nb_sq-2*a.r*b.r;

    c := Mrp.Element(
      r=((1-na_sq)*b.r+(1-nb_sq)*a.r-2*cross(b.r,a.r))/den);

  end product;

  encapsulated operator function inverse
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a;

    output Mrp.Element res;

  algorithm
    res := Mrp.Element(
      r=-a.r);

  end inverse;

  encapsulated operator function one
    import CogniPilot.Lie.SO3.Mrp;

    constant output Mrp.Element res;

  algorithm
    res := Mrp.Element(
      r={0,0,0});

  end one;

  encapsulated operator function equal
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a,b;

    input Real eps;

    output Boolean res;

  protected
    Real d[3];

  algorithm
    d := a.r-b.r;

    res := sqrt(
      d*d) < eps;

  end equal;

  encapsulated operator function shadow_if_necessary
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a;

    output Mrp.Element res;

  protected
    Real n_sq;

  algorithm
    n_sq := a.r*a.r;

    if(n_sq > 1) then
      res := Mrp.Element(
        r=-a.r/n_sq);

    else
      res := a;

    end if;

  end shadow_if_necessary;

  encapsulated operator function adjoint
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a;

    output Real R[3,3];

  algorithm
    R := Mrp.to_matrix(
      a);

  end adjoint;

  encapsulated operator function exp
    import CogniPilot.Lie.SO3;

    input SO3.Algebra.Element a;

    output SO3.Mrp.Element res;

  protected
    Real angle;

  algorithm
    angle := sqrt(
      a.r*a.r);

    if(angle > 1e-7) then
      res := SO3.Mrp.shadow_if_necessary(
        SO3.Mrp.Element(
          r=tan(angle/4)*a.r/angle));

    else
      res := SO3.Mrp.Element(
        r={0,0,0});

    end if;

  end exp;

  encapsulated operator function log
    import CogniPilot.Lie.SO3;

    input SO3.Mrp.Element a;

    output SO3.Algebra.Element res;

  protected
    Real n;

  algorithm
    n := sqrt(
      a.r*a.r);

    if(n > 1e-7) then
      res := SO3.Algebra.Element(
        r=4*atan(n)*a.r/n);

    else
      res := SO3.Algebra.Element(
        r={0,0,0});

    end if;

  end log;

  encapsulated operator function to_matrix
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a;

    output Real R[3,3];

  protected
    Real X[3,3],n_sq;

  algorithm
    X := skew(
      a.r);

    n_sq := a.r*a.r;

    R := identity(
      3)+(8*X*X+4*(1-n_sq)*X)/(1+n_sq)^2;

    annotation (
      Inline=true);

  end to_matrix;

  model Kinematics
    import CogniPilot.Lie.SO3.Mrp;

    input Mrp.Element a0;

    input Real[3] omega;

    output Mrp.Element a;

  protected
    Real v[3,1];

    Real n_sq;

    Real B[3,3];

  initial equation
    a.r=a0.r;

  equation
    v[1,1]=a.r[1];

    v[2,1]=a.r[2];

    v[3,1]=a.r[3];

    n_sq=a.r*a.r;

    B=0.25*((1-n_sq)*identity(
      3)+2*skew(
      a.r)+2*v*transpose(
      v));

    when(a.r*a.r > 1) then
      reinit(
        a.r,
        -a.r/(a.r*a.r));

    end when;

    der(
      a.r)=B*omega;

  end Kinematics;

end Group;
