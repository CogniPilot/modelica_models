within Lie2.SO3;

model Dcm
  
  type Element = Real[9];
    
  type BaseType = Base.Group(
    redeclare type Element = Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends BaseType;

  function one
    extends BaseType.one;
  algorithm
    res := {1, 0, 0, 0, 1, 0, 0, 0, 1};
    annotation(Inline = true);
  end one;

  function product
    extends BaseType.product;
  algorithm
    res := fromMatrix(toMatrix(a) * toMatrix(b));
    annotation(Inline = true);
  end product;
  
  function inv
    extends BaseType.inv;
  algorithm
    res := transpose(a);
    annotation(Inline = true);
  end inv;

  function log
    extends BaseType.log;
  protected
    Real x;
  algorithm
    x := acos((trace(a)-1)/2);
    if(x > 1e-7) then
      res := 4*atan(n)*a/n;
    else
      res := {0,0,0};
    end if;
    annotation(Inline = true);
  end log;

  function Dl
  extends BaseType.Dl;
  algorithm
    res := fromMatrix(Algebra.toMatrix(w)*toMatrix(a));
    annotation(
      Inline = true);
  end Dl;
  
  function Dr
  extends BaseType.Dr;
  algorithm
    res := fromMatrix(toMatrix(a)*Algebra.toMatrix(w));
    annotation(
      Inline = true);
  end Dr;
  
  function fromMatrix
    extends BaseType.fromMatrix;
  algorithm
    res := {a[1, 1], a[1, 2], a[1, 3],
            a[2, 1], a[2, 2], a[2, 3],
            a[3, 1], a[3, 2], a[3, 3]};
    annotation(Inline = true);
  end fromMatrix;
  
  function toMatrix
    extends BaseType.toMatrix;
  algorithm
    res := {{a[1], a[2], a[3]},
            {a[4], a[5], a[6]},
            {a[7], a[8], a[9]}};
    annotation(Inline = true);
  end toMatrix;
  
end Dcm;