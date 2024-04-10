within Lie2.Op;

package SO3Op

  type AlgebraOpFuncs = Op.Base.AlgebraOpFuncs(
    redeclare operator record OpType = AlgebraOp,
    redeclare type ElementType = SO3.Algebra.Element,
    redeclare type AlgebraType = SO3.Algebra,
    redeclare type ParamType = Real[3]
  );

  operator record AlgebraOp
    import Lie2.SO3;
    SO3.Algebra.Element elem;
  
    encapsulated operator 'constructor'
      import Lie2;
      function fromReal = Lie2.Op.SO3Op.AlgebraOpFuncs.fromReal;
      function fromElement = Lie2.Op.SO3Op.AlgebraOpFuncs.fromElement;
    end 'constructor';
  
    encapsulated operator '+'
      import Lie2;
      function add = Lie2.Op.SO3Op.AlgebraOpFuncs.add;
    end '+';
  
    //function exp = Algebra.exp(a = AlgebraElement(r = elem.r));
    
    end AlgebraOp;

  package Dcm
    
    type GroupOpFuncs = Op.Base.GroupOpFuncs(
      redeclare operator record OpType = GroupOp,
      redeclare type ElementType = SO3.Dcm.Group.Element,
      redeclare type GroupType = SO3.Dcm.Group,
      redeclare type ParamType = Real[3, 3]
    );
  
    operator record GroupOp
      import Lie2.SO3.Dcm.Group;
      Group.Element elem;
        
      encapsulated operator 'constructor'
        import Lie2;
        function fromReal = Lie2.Op.SO3Op.Dcm.GroupOpFuncs.fromReal;
        function fromElement = Lie2.Op.SO3Op.Dcm.GroupOpFuncs.fromElement;
      end 'constructor';
    
      encapsulated operator '*'
        import Lie2;
        function product = Lie2.Op.SO3Op.Dcm.GroupOpFuncs.product;
      end '*';
      
      function inv = Lie2.Op.SO3Op.Dcm.GroupOpFuncs.inv(a=GroupOp(elem));
      function log = Lie2.Op.SO3Op.Dcm.GroupOpFuncs.log;
      
    end GroupOp;
  
  end Dcm;

end SO3Op;