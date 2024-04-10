within Lie2.Op;

package R1Op  

  type GroupOpFuncs = Base.GroupOpFuncs(
    redeclare operator record OpType = R1Op.GroupOp,
    redeclare record ElementType = R1.Group.Element,
    redeclare model GroupType = R1.Group,
    redeclare type ParamType = Real[1]
  );
  
  type AlgebraOpFuncs = Base.AlgebraOpFuncs(
    redeclare operator record OpType = R1Op.AlgebraOp,
    redeclare record ElementType = R1.Algebra.Element,
    redeclare model AlgebraType = R1.Algebra,
    redeclare type ParamType = Real[1]
  );
  
  operator record GroupOp
    R1.Group.Element elem;
    
    encapsulated operator 'constructor'
      import Lie2.Op.R1Op;
      function fromReal = R1Op.GroupOpFuncs.fromReal;
      function fromElement = R1Op.GroupOpFuncs.fromElement;
    end 'constructor';
  
    encapsulated operator '*'
      import Lie2.Op.R1Op;
      function product = R1Op.GroupOpFuncs.product;
    end '*';
    
    //function inv = Lie2.Op.R1.OpFuncs.inv(a=GroupOp(elem));
    //function log = Lie2.Op.R1.OpFuncs.log;
    
  end GroupOp;
    
  operator record AlgebraOp
    R1.Algebra.Element elem;

    encapsulated operator 'constructor'
      import Lie2.Op.R1Op;
      function fromReal = R1Op.AlgebraOpFuncs.fromReal;
      function fromElement = R1Op.AlgebraOpFuncs.fromElement;
    end 'constructor';
  
    encapsulated operator '+'
      import Lie2.Op.R1Op;
      function add = R1Op.AlgebraOpFuncs.add;
    end '+';
  
    //function exp = Algebra.exp(a = AlgebraElement(r = elem.r));
    
  end AlgebraOp;

end R1Op;