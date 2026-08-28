within Vehicles.Interfaces;
function scheduledChannels
  "Hold one row of a time-indexed channel schedule"
  input Real now_s(unit = "s");
  input Real activationTime_s[:](each unit = "s")
    "Strictly ascending activation times";
  input Real channelValue[size(activationTime_s, 1), :]
    "One row of channel values per activation time";
  output Real value[size(channelValue, 2)] "Row currently in force";
algorithm
  value := channelValue[
    Vehicles.Interfaces.activeScheduleRow(now_s, activationTime_s), :];
  annotation(Documentation(info = "<html>
    <p>The lookup is a function of time alone, so the held row does not depend
    on when sampling started or on a retained index, and the schedule composes
    into an ordinary equation rather than needing a clock of its own.</p>
  </html>"));
end scheduledChannels;
