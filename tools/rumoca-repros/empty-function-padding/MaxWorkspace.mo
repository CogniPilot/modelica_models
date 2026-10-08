within;

model MaxWorkspace
  function evaluate
    input Real columns[:, :];
    output Real result[size(columns, 1), size(columns, 1)];
  protected
    Real work[max(size(columns, 1), size(columns, 2)), size(columns, 1)];
  algorithm
    work := zeros(max(size(columns, 1), size(columns, 2)), size(columns, 1));
    work[1, 1] := columns[1, 1];
    result := work[1:size(columns, 1), :];
  end evaluate;
  Real columns[3, 5] = fill(time, 3, 5);
  Real result[3, 3] = evaluate(columns);
end MaxWorkspace;
