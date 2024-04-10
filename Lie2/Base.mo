within Lie2;

package Base

  model Algebra
  
    replaceable type Element = Real;
  
    partial function add
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end add;
  
    partial function subtract
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end subtract;
  
    partial function bracket
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end bracket;
  
    partial function scalar_multiply
      input Real a;
      input Element b;
      output Element res;
      annotation(Inline = true);
    end scalar_multiply;
  
    partial function exp
      replaceable type GroupElement = Element;
      input Element a;
      output GroupElement res;
      annotation(Inline = true);
    end exp;
  
  end Algebra;
  model Group
  
    replaceable type Element = Real;
  
    replaceable type AlgebraElement = Element;
  
    partial function product
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end product;
  
    partial function inv
      input Element a;
      output Element res;
      annotation(Inline = true);
    end inv;
    
    partial function log
      input Element a;
      output AlgebraElement res;
      annotation(Inline = true);
    end log;
  
  end Group;
end Base;