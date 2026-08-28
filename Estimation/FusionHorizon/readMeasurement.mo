within Estimation.FusionHorizon;

function readMeasurement
  "Select one queued measurement without a dynamic index"
  input Real queue[:, :]
    "One aiding source's delayed-measurement FIFO, one packed sample per row";
  input Integer slot
    "Row to read; any value outside 1..size(queue,1) reads zero, which is how
     a tick with an empty queue reads nothing without a branch";
  output Real row[size(queue, 2)];
protected
  Real hit;
algorithm
  // The same masked fixed-length walk Estimation.FusionHorizon.readRow uses on
  // the delta ring, and for the same two reasons. The code generator refuses a
  // dynamic array index it cannot prove in range, which for a ring buffer is
  // the right refusal; and a constant-time read makes the worst case the
  // measured case, which is the only kind of worst case a flight timing record
  // can be written against.
  //
  // It is generic in the row width rather than fixed at DeltaLength because
  // the five aiding sources pack to five different widths and one kernel over
  // all of them is one kernel to get right.
  row := zeros(size(queue, 2));
  for k in 1:size(queue, 1) loop
    hit := if k == slot then 1.0 else 0.0;
    row := row + hit * queue[k, :];
  end for;
end readMeasurement;
