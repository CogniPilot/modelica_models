within Lie;

package R2

  type Group = Lie.R1.Group(redeclare type ParamType=Real[2]);
  type Algebra = Lie.R1.Algebra(redeclare type ParamType=Real[2]);

end R2;