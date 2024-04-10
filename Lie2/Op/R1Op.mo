within Lie2.Op;

package R1Op  

  model GroupOpFuncs = Base.GroupOpFuncs(
    redeclare record ElementType = R1.Group.Element,
    redeclare model GroupType = R1.Group,
    redeclare type ParamType = Real[1]
  );
  
  model AlgebraOpFuncs = Base.AlgebraOpFuncs(
    redeclare record ElementType = R1.Algebra.Element,
    redeclare model AlgebraType = R1.Algebra,
    redeclare type ParamType = Real[1]
  );
   
  operator record GroupOp
    R1.Group.Element elem;
    
    encapsulated operator 'constructor'
      import Lie2;
      function fromReal = Lie2.Op.R1Op.GroupOpFuncs.fromReal(redeclare operator record OpType = Lie2.Op.R1Op.GroupOp);
      function fromElement = Lie2.Op.R1Op.GroupOpFuncs.fromElement(redeclare operator record OpType = GroupOp);
    end 'constructor';
  
    encapsulated operator '*'
      import Lie2;
      function product = R1Op.GroupOpFuncs.product(redeclare operator record OpType = Lie2.Op.R1Op.GroupOp);
    end '*';
    
    //function inv = Lie2.Op.R1.OpFuncs.inv(a=GroupOp(elem));
    //function log = Lie2.Op.R1.OpFuncs.log;
    
  end GroupOp;
    
  operator record AlgebraOp
    R1.Algebra.Element elem;

    model Funcs = Base.AlgebraOpFuncs(
      redeclare record ElementType = R1.Algebra.Element,
      redeclare model AlgebraType = R1.Algebra,
      redeclare type ParamType = Real[1]
    );
  
    encapsulated operator 'constructor'
      import Lie2;
      function fromReal = Lie2.Op.R1Op.AlgebraOp.Funcs.fromReal(redeclare operator record OpType = Lie2.Op.R1Op.AlgebraOp);
      function fromElement = Lie2.Op.R1Op.AlgebraOp.Funcs.fromElement(redeclare operator record OpType = Lie2.Op.R1Op.AlgebraOp);
    end 'constructor';
  
    encapsulated operator '+'
      import Lie2;
      function add = Lie2.Op.R1Op.AlgebraOpFuncs.add(redeclare operator record OpType = Lie2.Op.R1Op.AlgebraOp);
    end '+';
  
    //function exp = Algebra.exp(a = AlgebraElement(r = elem.r));
    
  end AlgebraOp;

end R1Op;