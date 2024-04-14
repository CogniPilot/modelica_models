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
    res := fromMatrix(transpose(toMatrix(a)));
    annotation(Inline = true);
  end inv;

  function log
    extends BaseType.log;
  protected
    Real x, c1, R[3, 3];
  algorithm
    R := toMatrix(a);
    x := acos((Math.trace(R) -1)/2);
    if(abs(x) > Constants.eps) then
      c1 := x/(2*sin(x));
    else
      c1 := 1/2 + x^2/12 + 7*x^4/720 + 21*x^6/30240;
    end if;
    res := c1 * Algebra.fromMatrix(R - transpose(R));
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

  function equal
    extends BaseType.equal;
  algorithm
    res := Math.norm2(log(product(a, inv(b)))) < eps;
    annotation(Inline = true);
  end equal;
  
  function normError
    extends BaseType.normError;
  algorithm
    res := max(abs({
      a[1:3]*a[1:3] - 1,
      a[4:6]*a[4:6] - 1,
      a[7:9]*a[7:9] - 1,
      a[1:3]*a[4:6],
      a[1:3]*a[7:9],
      a[4:6]*a[7:9]}));
    annotation(Inline = true);
  end normError;

  function normalize "TODO: requires expensive inverse"
    extends BaseType.normalize;
  algorithm
    res := a;
    annotation(Inline = true);
  end normalize;
  
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