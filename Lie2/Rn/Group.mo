within Lie2.Rn;

model Group

  type Element = Real[n];

  type BaseType = Base.Group(
    redeclare type Element = Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends BaseType;
   
  function product
    extends BaseType.product;
  algorithm
    res := a + b;
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
  algorithm
    res := -a;
    annotation(Inline = true);
  end log;

end Group;