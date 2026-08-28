within Vehicles.Interfaces;
function activeScheduleRow
  "Locate the last schedule row whose activation time has passed"
  input Real now_s(unit = "s");
  input Real activationTime_s[:](each unit = "s")
    "Strictly ascending activation times";
  output Integer row(min = 1, max = size(activationTime_s, 1));
algorithm
  row := 1;
  for index in 1:size(activationTime_s, 1) loop
    if now_s >= activationTime_s[index] then
      row := index;
    end if;
  end for;
  annotation(Documentation(info = "<html>
    <p>A zero-order hold over a time-indexed table. Scanning the whole table
    makes the result a function of time alone, so it does not depend on when
    sampling started or on a retained row index.</p>
  </html>"));
end activeScheduleRow;
