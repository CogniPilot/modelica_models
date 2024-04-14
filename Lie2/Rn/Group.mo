within Lie2.Rn;

model Group
  type Element = Real[n];
  type BaseType = Base.Group(redeclare type Element = Element, redeclare type AlgebraElement = Algebra.Element);
  extends BaseType;

  function product
    extends BaseType.product;
  algorithm
    res := a + b;
    annotation(
      Inline = true);
  end product;

  function inv
    extends BaseType.inv;
  algorithm
    res := -a;
    annotation(
      Inline = true);
  end inv;

  function log
    extends BaseType.log;
  algorithm
    res := a;
    annotation(
      Inline = true);
  end log;

  function exp
    extends BaseType.exp;
  algorithm
    res := a;
    annotation(
      Inline = true);
  end exp;

  function equal
    extends BaseType.equal;
  algorithm
    res := Math.norm2(log(product(a, inv(b)))) < eps;
    annotation(Inline = true);
  end equal;

  function fromMatrix
    extends BaseType.fromMatrix;
  algorithm
    res := { a[i, n+1] for i in 1:n };
    annotation(Inline = true);
  end fromMatrix;

  function toMatrix
    extends BaseType.toMatrix;
  algorithm
    res := cat(2,
      cat(1, identity(n), zeros(1, n)),
      cat(1, matrix(a), ones(1, 1)));
    annotation(Inline = true);
  end toMatrix;
end Group;