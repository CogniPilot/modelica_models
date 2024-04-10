within Lie2.Op;

package Base
  model GroupOpFuncs
  
    operator record DefaultOp
      EmptyElement elem;
    end DefaultOp;
    
    record DefaultElement
      Real r;
    end DefaultElement;
  
    replaceable operator record OpType = DefaultOp;
    replaceable record ElementType = DefaultElement;
    replaceable model GroupType = Base.Group;
    replaceable type ParamType = Real[1];
  
    function fromReal
      input ParamType r;
      output OpType res = OpType(elem=ElementType(r=r));
      annotation(Inline = true);
    end fromReal;
    
    function fromElement
      input ElementType elem;
      output OpType res = OpType(elem=ElementType(r=elem.r));
      annotation(Inline = true);
    end fromElement;
    
    function product
      input OpType a, b;
      output OpType c;
    algorithm
      c := OpType(elem=GroupType.product(a.elem, b.elem));
      annotation(Inline = true);
    end product;
    
    function inv
      input OpType a;
      output OpType res = GroupOp(elem=GroupType.inv(a.elem));
      annotation(Inline = true);
    end inv;
    
    function log
      input OpType a = GroupOp(elem=elem);
      output OpType res = GroupOp(elem=GroupType.log(a.elem));
      annotation(Inline = true);
    end log;
    
  end GroupOpFuncs;
  
  model AlgebraOpFuncs
  
    operator record DefaultOp
      EmptyElement elem;
    end DefaultOp;
    
    record DefaultElement
      Real r;
    end DefaultElement;
  
    replaceable operator record OpType = DefaultOp;
    replaceable record ElementType = DefaultElement;
    replaceable model AlgebraType = Base.Algebra;
    replaceable type ParamType = Real[1];
  
    function fromReal
      input ParamType r;
      output OpType res = OpType(elem=ElementType(r=r));
      annotation(Inline = true);
    end fromReal;
    
    function fromElement
      input ElementType elem;
      output OpType res = OpType(elem=ElementType(r=elem.r));
      annotation(Inline = true);
    end fromElement;
    
    function add
      input OpType a, b;
      output OpType c;
    algorithm
      c := OpType(elem=AlgebraType.add(a.elem, b.elem));
      annotation(Inline = true);
    end add;
    
  end AlgebraOpFuncs;
end Base;