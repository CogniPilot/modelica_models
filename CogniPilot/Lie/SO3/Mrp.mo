within CogniPilot.Lie.SO3;
operator record Mrp
  extends SO3.Group;
  encapsulated operator record Element
    import CogniPilot.Lie;
    extends Lie.Group.Element;
    Real r[3];
    encapsulated operator '*'
      function product
        input Mrp.Element a,b;
        output Mrp.Element res;
      algorithm
        res := Mrp.product(
          a,
          b);
      end product;
    end '*';
    encapsulated operator function Ad=Lie.SO3.Mrp.adjoint(
      a=Lie.SO3.Mrp.Element(
        r));
    encapsulated operator function log=Lie.SO3.Mrp.log(
      a=Lie.SO3.Mrp.Element(
        r));
    encapsulated operator function to_matrix=Lie.SO3.Mrp.to_matrix(
      a=Lie.SO3.Mrp.Element(
        r));
  end Element;
  encapsulated operator function product
    input Mrp.Element a,b;
    output Mrp.Element c;
  protected
    Real na_sq,nb_sq,den,res[3];
  algorithm
    na_sq := a.r*a.r;
    nb_sq := b.r*b.r;
    den := 1+na_sq+nb_sq-2*a.r*b.r;
    c := Mrp.Element(
      r=((1-na_sq)*b.r+(1-nb_sq)*a.r-2*cross(b.r,a.r))/den);
  end product;
  encapsulated operator function inverse
    input Mrp.Element r1;
    output Mrp.Element res;
  algorithm
    res := Mrp(
      r=-r1.r);
  end inverse;
  encapsulated operator function one
    output Mrp.Element res;
  algorithm
    res := Mrp.Element(
      r={0,0,0});
  end one;
  encapsulated operator function shadow_if_necessary
    input Mrp.Element r1;
    output Mrp.Element res;
  protected
    Real n_sq;
  algorithm
    n_sq := r1.r*r1.r;
    if(n_sq > 1) then
      res := Mrp.Element(
        r=-r1.r/n_sq);
    else
      res := r1;
    end if;
  end shadow_if_necessary;
  encapsulated operator function adjoint
    input Mrp.Element a;
    output Real R[3,3];
  algorithm
    R := Mrp.to_matrix(
      a);
  end adjoint;
  encapsulated operator function exp
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
      Inline=false);
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
end Mrp;
