within Lie2;

package Base

  model Algebra

    record Element
    end Element;
  
    replaceable type GroupElement = Element;
    replaceable type ElementType = Element;
  
    partial function add
      input ElementType a, b;
      output ElementType res;
      annotation(Inline = true);
    end add;
  
    partial function subtract
      input ElementType a, b;
      output ElementType res;
      annotation(Inline = true);
    end subtract;
  
    partial function bracket
      input ElementType a, b;
      output ElementType res;
      annotation(Inline = true);
    end bracket;
  
    partial function scalar_multiply
      input Real a;
      input ElementType b;
      output ElementType res;
      annotation(Inline = true);
    end scalar_multiply;
  
    partial function exp
      input ElementType a;
      output GroupElement res;
      annotation(Inline = true);
    end exp;
  
  end Algebra;
  model Group

    record Element
    end Element;
  
    replaceable type AlgebraElement = Element;
    replaceable type ElementType = Element;
  
    partial function product
      input ElementType a, b;
      output ElementType res;
      annotation(Inline = true);
    end product;
  
    partial function inv
      input ElementType a;
      output ElementType res;
      annotation(Inline = true);
    end inv;
    
    partial function log
      input ElementType a;
      output AlgebraElement res;
      annotation(Inline = true);
    end log;
  
  end Group;
end Base;