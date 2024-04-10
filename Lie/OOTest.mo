package OOTest
   
operator record Base
  
  Real r[1];
  
  encapsulated operator '*'
    import OOTest.Base;
    
    function op_times = Base.product(a, b);
    
  end '*';

    function product
      input Base a, b;
      output Base res;
    algorithm
      res := Base(r=a.r + b.r);
    end product;
  
end Base;

  model Test
    Derived1 a(r={1}), b(r={1}), c;
  equation
    c = a*b;
  end Test;
  
  operator record Derived1
  
    extends Base;
  
  encapsulated operator '*'
      import OOTest.Derived1;
      
      function op_times = Derived1.product(a, b);
      
    end '*';
  
  
    redeclare function product
      input Base a, b;
      output Base res;
    algorithm
      res := Base(r=2*(a.r + b.r));
    end product;
  
  end Derived1;
end OOTest;