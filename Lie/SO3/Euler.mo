within Lie.SO3;
operator record Euler
  type RotationType=enumeration(Body
    "Body",Space
    "Space");

  extends Lie.Group;

  Real angles[3];

  parameter Integer axisSeq[3];

  parameter RotationType rotType;

  function toMatrix
    "Euler Sequence to Matrix"
    input Euler a;

    output Real[3,3] res;

  algorithm
    res := identity(
      3);

    for i in 1:3 loop
      if(a.rotType == RotationType.Space) then
        res := rotationMatrix(
          a.angles[i],
          a.axisSeq[i])*res;

      elseif(a.rotType == RotationType.Body) then
        res := res*rotationMatrix(
          a.angles[i],
          a.axisSeq[i]);

      end if;

    end for;

    annotation (
      Inline=true);

  end toMatrix;

  function rotationMatrix
    input Real x;

    input Integer axis;

    output Real[3,3] res;

  algorithm
    assert(
      axis > 0 and axis < 4,
      "axis out of range");

    if(axis == 1) then
      res := {{1,0,0},{0,cos(x),-sin(x)},{0,sin(x),cos(x)}};

    elseif(axis == 2) then
      res := {{cos(x),0,sin(x)},{0,1,0},{-sin(x),0,cos(x)}};

    elseif(axis == 3) then
      res := {{cos(x),-sin(x),0},{sin(x),cos(x),0},{0,0,1}};

    end if;

    annotation (
      Inline=true);

  end rotationMatrix;

end Euler;
