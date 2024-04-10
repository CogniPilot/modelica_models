within Lie.R1;

operator record Group
  extends Lie.Group(
    redeclare type ParamType = Real[1],
    //redeclare type GroupType = Lie.R1.Group,
    //redeclare type AlgebraType = Lie.R1.Algebra,
    redeclare function product = Lie.R1.Group.product
    );

  ParamType r;

  function log
    "Log map."
    input Group a;
    
    output AlgebraType res;
  
  
  algorithm
    
    res.r := phi.r;
  
    annotation (
      Inline=true);
  
  end log;
 
  function product
  
    input Group a;

    input Group b;
    
    output Group res;
    
  algorithm
  
    res.r := a.r + b.r;

  end product;
  
end Group;
