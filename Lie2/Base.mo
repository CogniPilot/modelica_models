within Lie2;

package Base

  model Algebra
  
    replaceable type Element = Real;
  
    function add
      input Element a, b;
      output Element res = a + b;
      annotation(Inline = true);
    end add;
  
    function subtract
      input Element a, b;
      output Element res = a - b;
      annotation(Inline = true);
    end subtract;
  
    partial function bracket
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end bracket;
  
    function scalarMultiply
      input Real a;
      input Element b;
      output Element res = a * b;
      annotation(Inline = true);
    end scalarMultiply;
  
    partial function exp
      replaceable type GroupElement = Element;
      input Element a;
      output GroupElement res;
      annotation(Inline = true);
    end exp;
  
    partial function ad
      input Element a;
      output Real [:, :] res;
      annotation(Inline = true);
    end ad;
    
    partial function fromMatrix
      input Real[:, :] a;
      output Element res;
      annotation(Inline = true);
    end fromMatrix;
  
    partial function toMatrix
      input Element a;
      output Real [:, :] res;
      annotation(Inline = true);
    end toMatrix;
  
  end Algebra;
  model Group
  
    replaceable     type Element = Real;
  
    replaceable     type AlgebraElement = Real;
  
    partial function identity
      output Element res;
      annotation(Inline = true);
    end identity;
  
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
  
    partial function Ad
      input Element a;
      output Real [:, :] res;
      annotation(Inline = true);
    end Ad;
  
    partial function fromMatrix
      input Real[:, :] a;
      output Element res;
      annotation(Inline = true);
    end fromMatrix;
  
    partial function toMatrix
      input Element a;
      output Real [:, :] res;
      annotation(Inline = true);
    end toMatrix;
    
  end Group;
end Base;