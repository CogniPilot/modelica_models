within Lie2.SO3;

model Quat

  type Element = Real[4];
    
  type BaseType = Base.Group(
    redeclare type Element = Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends BaseType;

  function one
    extends BaseType.one;
  algorithm
    res := {1, 0, 0, 0};
    annotation(Inline = true);
  end one;
   
  function product // TODO
    extends BaseType.product;
  protected
    Real na_sq,nb_sq,den,res[3];
  algorithm
    na_sq := a*a;
    nb_sq := b*b;
    den := 1+na_sq*nb_sq-2*a*b;
    res := ((1-na_sq)*b+(1-nb_sq)*a-2*cross(b,a))/den;
    annotation(Inline = true);
  end product;
  
  function inv // TODO
    extends BaseType.inv;
  algorithm
    res := -a;
    annotation(Inline = true);
  end inv;

  function log // TODO
    extends BaseType.log;
  protected
    Real n;
  algorithm
    res := -a;
    n := sqrt(a*a);
    if(n > 1e-7) then
      res := 4*atan(n)*a/n;
    else
      res := {0,0,0};
    end if;
    annotation(Inline = true);
  end log;
  
  function toMatrix // TODO
    extends BaseType.toMatrix;
  protected
    Real X[3,3],n_sq;
  algorithm
    X := skew(a);
    n_sq := a*a;
    res := identity(3)+(8*X*X+4*(1-n_sq)*X)/(1+n_sq)^2;
    annotation(Inline = true);
  end toMatrix;

end Quat;