within Lie2.SO3;

package Dcm

  model Group
  
    type Element = Real[3, 3];
      
    type BaseType = Base.Group(
      redeclare type Element = Element,
      redeclare type AlgebraElement = Algebra.Element);
    
    extends BaseType;
     
    function product
      extends BaseType.product;
    algorithm
      res := a * b;
      annotation(Inline = true);
    end product;
    
    function inv
      extends BaseType.inv;
    algorithm
      res := transpose(a);
      annotation(Inline = true);
    end inv;
  
    function log // TODO
      extends BaseType.log;
    algorithm
      res := Element(-a);
      annotation(Inline = true);
    end log;
  
  end Group;

end Dcm;