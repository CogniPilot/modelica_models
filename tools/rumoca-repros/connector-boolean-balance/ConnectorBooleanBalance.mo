within;
package ConnectorBooleanBalance
  "Boolean components of a sub-block's input connector are not counted"

  connector SampleInput = input Sample;

  record Sample
    Boolean valid;
    Boolean fresh;
    Real timestamp_s;
    Real value_m[3];
  end Sample;

  block Inner "Consumes the connector"
    SampleInput s;
    output Real y;
  equation
    y = s.timestamp_s + sum(s.value_m)
      + (if s.valid then 1.0 else 0.0)
      + (if s.fresh then 1.0 else 0.0);
  end Inner;

  block Outer "Passes one whole record through to the sub-block"
    SampleInput s;
    output Real y;
    Inner inner1;
  equation
    inner1.s = s;
    y = inner1.y;
  end Outer;

  annotation(Documentation(info="<html>
    <p><code>Outer</code> is reported unbalanced by exactly two equations,
    which is the number of Boolean components on <code>Sample</code>. Removing
    the two Booleans from the record makes it balance.</p>
  </html>"));
end ConnectorBooleanBalance;
