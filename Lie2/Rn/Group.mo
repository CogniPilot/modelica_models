within Lie2.Rn;

model Group

  record Element
    Real r[n];
  end Element;
  
  import Lie2;
  
  type Base = Lie2.Base.Group(
    redeclare type ElementType = Group.Element,
    redeclare type AlgebraElement = Algebra.Element);
  
  extends Base;
   
  function product
    extends Base.product;
  algorithm
    res := Element(a.r + b.r);
  end product;
  
  function inverse
    extends Base.inverse;
  algorithm
    res := Element(-a.r);
  end inverse;

  function log
    extends Base.log;
  algorithm
    res := Element(-a.r);
  end log;

end Group;