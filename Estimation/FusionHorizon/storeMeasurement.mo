within Estimation.FusionHorizon;

function storeMeasurement
  "Write one packed sample into a measurement queue and return the queue"
  input Real queue[:, :];
  input Integer slot
    "Row to write; any value outside 1..size(queue,1) writes nothing, which is
     how a tick that admits no measurement stores nothing without a branch";
  input Real row[size(queue, 2)];
  output Real updated[size(queue, 1), size(queue, 2)];
protected
  Real hit;
algorithm
  // Masked and fixed-length for the reasons recorded in
  // Estimation.FusionHorizon.storeMeasurement's read counterpart. Generic in
  // the row width so the five aiding sources share one store.
  for k in 1:size(queue, 1) loop
    hit := if k == slot then 1.0 else 0.0;
    updated[k, :] := hit * row + (1.0 - hit) * queue[k, :];
  end for;
end storeMeasurement;
