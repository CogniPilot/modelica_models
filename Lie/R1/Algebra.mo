within Lie.R1;

operator record Algebra
  extends Lie.Algebra(redeclare type ParamType=Real[1]);
  final type GroupType = Lie.R1.Group(redeclare type ParamType=ParamType);
  final type AlgebraType = Lie.R1.Group(redeclare type ParamType=ParamType);

  ParamType r;

  encapsulated operator '+'
    "Add two Lie Algebra elements"
    import Lie.Algebra;
    
    function add
      input Algebra a;
  
      input Algebra b;
  
      output Algebra res;
  
    algorithm
      res.r := a.r+b.r;
  
      annotation (
        Inline=true);
  
    end add;
  
  end '+';

  function exp
    "Exponential map."
    input AlgebraType phi;
  
    output GroupType res;
  
  algorithm
    
    res.r := phi.r;
  
    annotation (
      Inline=true);
  
  end exp;


end Algebra;
