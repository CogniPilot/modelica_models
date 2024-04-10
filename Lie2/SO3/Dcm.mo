within Lie2.SO3;

package Dcm

  model Group
  
    record Element
      Real r[3, 3];
    end Element;
      
    type BaseType = Base.Group(
      redeclare type ElementType = Group.Element,
      redeclare type AlgebraElement = Algebra.Element);
    
    extends BaseType;
     
    function product
      extends BaseType.product;
    algorithm
      res := Element(a.r * b.r);
      annotation(Inline = true);
    end product;
    
    function inv
      extends BaseType.inv;
    algorithm
      res := Element(transpose(a.r));
      annotation(Inline = true);
    end inv;
  
    function log // TODO
      extends BaseType.log;
    algorithm
      res := Element(-a.r);
      annotation(Inline = true);
    end log;
  
  end Group;

end Dcm;