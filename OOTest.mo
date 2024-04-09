encapsulated package OOTest

  model Test
  equation
  
  end Test;
   
operator record Base

  encapsulated operator 'constructor'
    import Base;
    
  end 'constructor';
  
  encapsulated operator '+'
    import Base;
    function add
      input Real x;
      input Real y;
      output Real z;
    algorithm
      z := x + y;
    end add;
  end '+';

  encapsulated operator '-'
    import Base;
    function add
      input Real x;
      input Real y;
      output Real z;
    algorithm
      z := x + y;
    end add;
  end '-';

end Base;

  encapsulated operator record Derived1
    import OOTest;
    extends OOTest.Base;
  end Derived1;
end OOTest;