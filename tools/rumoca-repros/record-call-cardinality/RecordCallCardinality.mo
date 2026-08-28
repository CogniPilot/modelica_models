within;
package RecordCallCardinality
  "One record-valued call must reach the DAE as exactly one call owner"

  record Pair
    Real left;
    Real right;
  end Pair;

  function expensive "Stands in for any call worth not evaluating three times"
    input Real value;
    output Real result;
  algorithm
    result := value * value + value;
  end expensive;

  function makePair "Assigns one field from a call, then reads that field twice"
    input Real value;
    output Pair pair;
  algorithm
    pair.left := expensive(value);
    pair.right := pair.left + pair.left;
  end makePair;

  model Outer
    Real u;
    Real y;
  protected
    Pair pair;
  equation
    u = time;
    pair = makePair(u);
    y = pair.left + pair.right;
  end Outer;

  annotation(Documentation(info="<html>
    <p>Compiled with <code>--emit dae-json</code>, the DAE expression arena
    should hold exactly one call owner for <code>expensive</code>. Today it
    holds three.</p>
  </html>"));
end RecordCallCardinality;
