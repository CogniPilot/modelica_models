within Lie2.Rn;

model Group

  record Element
    Real r[n];
  end Element;
    
  type BaseType = Base.Group(
    redeclare type ElementType = Group.Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends BaseType;
   
  function product
    extends BaseType.product;
  algorithm
    res := Element(a.r + b.r);
    annotation(Inline = true);
  end product;
  
  function inv
    extends BaseType.inv;
  algorithm
    res := Element(-a.r);
    annotation(Inline = true);
  end inv;

  function log
    extends BaseType.log;
  algorithm
    res := Element(-a.r);
    annotation(Inline = true);
  end log;

end Group;