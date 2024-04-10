within Lie;

package R3

  type Group = Lie.R1.Group(redeclare type ParamType=Real[3]);
  type Algebra = Lie.R1.Algebra(redeclare type ParamType=Real[3]);

end R3;