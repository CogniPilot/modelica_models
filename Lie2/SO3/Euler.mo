within Lie2.SO3;

model Euler

  replaceable constant Integer[3] sequence = {3, 2, 1};
  replaceable constant Boolean body = true; 
  
  type Element = Real[3];
    
  type BaseType = Base.Group(
    redeclare type Element = Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends BaseType;

  function one
    extends BaseType.one;
  algorithm
    res := {0, 0, 0};
    annotation(Inline = true);
  end one;
   
  function product
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
  
  function inv
    extends BaseType.inv;
  algorithm
    res := -a;
    annotation(Inline = true);
  end inv;

  function log
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

  function axisRotationMatrix
    input Real x;
    input Integer axis;
    output Real[3,3] res;
  algorithm
    assert(axis > 0 and axis < 4, "axis out of range");
    if(axis == 1) then
      res := {{1,0,0},{0,cos(x),-sin(x)},{0,sin(x),cos(x)}};
    elseif(axis == 2) then
      res := {{cos(x),0,sin(x)},{0,1,0},{-sin(x),0,cos(x)}};
    elseif(axis == 3) then
      res := {{cos(x),-sin(x),0},{sin(x),cos(x),0},{0,0,1}};
    end if;
    annotation (Inline=true);
  end axisRotationMatrix;

  function normError
    extends BaseType.normError(redeclare constant output Real res = 0);
  algorithm
    res := 0;
    annotation(Inline = true);
  end normError;

  function normalize
    extends BaseType.normalize;
  algorithm
    res := a;
    annotation(Inline = true);
  end normalize;

  function toMatrix
    extends BaseType.toMatrix;
  algorithm
    res := identity(3);
    for i in 1:3 loop
      if(body == false) then
        res := axisRotationMatrix(
          a[i],
          sequence[i])*res;
      elseif(body == true) then
        res := res*axisRotationMatrix(
          a[i],
          sequence[i]);
      end if;
    end for;
    annotation (Inline=true);
  end toMatrix;
  
end Euler;