within Lie2.SO2;

model Algebra
  record Element
    Real r[n];
  end Element;

  import Lie2;
  type Base = Lie2.Base.Algebra(redeclare type ElementType = Element, redeclare type GroupElement = Group.Element);
  extends Base;

  function add
    extends Base.add;
  algorithm
    res := Element(a.r + b.r);
  end add;

  function exp
    extends Base.exp;
  algorithm
    res := Group.Element(a.r);
  end exp;
end Algebra;