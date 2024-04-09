within Lie2;

package Base
  model Group

    record Element
    end Element;
  
    replaceable type AlgebraElement = Element;
    replaceable type ElementType = Element;
  
    partial function product
      input ElementType a, b;
      output ElementType res;
    end product;
  
    partial function inverse
      input ElementType a;
      output ElementType res;
    end inverse;
    
    partial function log
      input ElementType a;
      output AlgebraElement res;
    end log;
  
  end Group;

  model Algebra

    record Element
    end Element;
  
    replaceable type GroupElement = Element;
    replaceable type ElementType = Element;
  
    partial function add
      input ElementType a, b;
      output ElementType res;
    end add;
  
    partial function subtract
      input ElementType a, b;
      output ElementType res;
    end subtract;
  
    partial function bracket
      input ElementType a, b;
      output ElementType res;
    end bracket;
  
    partial function scalar_multiply
      input Real a;
      input ElementType b;
      output ElementType res;
    end scalar_multiply;
  
    partial function exp
      input ElementType a;
      output GroupElement res;
    end exp;
  
  end Algebra;
end Base;