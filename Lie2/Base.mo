within Lie2;

package Base

  model Algebra
  
    replaceable type Element = Real;
  
    function add "Add two elements"
      input Element a, b;
      output Element res = a + b;
      annotation(Inline = true);
    end add;
  
    function subtract "Subtract two elements"
      input Element a, b;
      output Element res = a - b;
      annotation(Inline = true);
    end subtract;
  
    partial function bracket "Lie algebra bracket operator"
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end bracket;
  
    function scalarMultiply "Scalar multiply from vector space"
      input Real a;
      input Element b;
      output Element res = a * b;
      annotation(Inline = true);
    end scalarMultiply;
  
    partial function exp "Lie exponential map"
      replaceable type GroupElement = Element;
      input Element a;
      output GroupElement res;
      annotation(Inline = true);
    end exp;
  
    partial function ad "adjoint of Lie Algebra"
      input Element a;
      output Real [:, :] res;
      annotation(Inline = true);
    end ad;
  
    partial function Jl "Left Jacobian of algebra parameterization a wrt tangent vector w"
      input Element a;
      input Element w;
      output Real [:, :] res;
      annotation(Inline = true);
    end Jl;
  
    partial function Jr "Right Jacobian of algebra parameterization a wrt tangent vector w"
      input Element a;
      input Element w;
      output Real [:, :] res;
      annotation(Inline = true);
    end Jr;
  
    function equal
      input Element a;
      input Element b;
      input Real eps = Constants.eps;
      output Boolean res;
    protected
      Element e;
    algorithm
      e := a - b;
      res := e * e < eps;
      annotation(Inline = true);
    end equal;
  
  
  
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
  
    partial function one "Identity element for group product"
      output Element res;
      annotation(Inline = true);
    end one;
  
    partial function product "Product of group elements"
      input Element a, b;
      output Element res;
      annotation(Inline = true);
    end product;
  
    partial function inv "Inverse of group element"
      input Element a;
      output Element res;
      annotation(Inline = true);
    end inv;
    
    partial function log "Lie logarithm map from group to algebra"
      input Element a;
      output AlgebraElement res;
      annotation(Inline = true);
    end log;
  
    partial function Ad "Group Adjoint operator Ad(A) B = A B A^{-1}"
      input Element a;
      output Real [:, :] res;
      annotation(Inline = true);
    end Ad;
    
    function equal
      input Element a;
      input Element b;
      input Real eps = Constants.eps;
      output Boolean res;
    protected
      Element e;
    algorithm
      e := a - b;
      res := e * e < eps;
      annotation(Inline = true);
    end equal;
  
    partial function Dl "Left derivative of group parameterization a wrt tangent vector w"
      input Element a;
      input AlgebraElement w;
      output Real[:] res;
      annotation(Inline = true);
    end Dl;
  
    partial function Dr "Right derivative of group parameterization a wrt tangent vector w"
      input Element a;
      input AlgebraElement w;
      output Real[:] res;
      annotation(Inline = true);
    end Dr;
  
    partial function normError
      input Element a;
      output Real res;
      annotation(Inline = true);
    end normError;
  
    partial function normalize
      input Element a;
      output Element res;
      annotation(Inline = true);
    end normalize;
  
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