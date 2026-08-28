within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block ScriptedTransmitter
  "Deterministic pilot transmitter replayed from a table of channel settings"

  parameter Integer channelCount(min = 5) = 5
    "{roll, pitch, yaw, throttle, mode switch}";
  parameter Integer eventCount(min = 1) = 1
    "Number of populated rows of the script";
  parameter Real eventTime_s[eventCount](each unit = "s") = {0.0}
    "Ascending times at which the pilot moves to the matching row";
  parameter Real channelPwm_us[eventCount, channelCount] = {
    {1500.0, 1500.0, 1500.0, 1000.0, 1900.0}}
    "Channel pulse widths held from each event time";

  output Real channelPwm_us_out[channelCount]
    "Pulse widths currently transmitted";

equation
  assert(eventTime_s[1] <= 0.0,
    "ScriptedTransmitter script must define the channels from time zero");
  for index in 2:eventCount loop
    assert(eventTime_s[index] > eventTime_s[index - 1],
      "ScriptedTransmitter event times must be strictly ascending");
  end for;

  channelPwm_us_out = Vehicles.Interfaces.scheduledChannels(
    time, eventTime_s, channelPwm_us);

  annotation(Documentation(info = "<html>
    <p>Models the pilot's radio as what a receiver actually delivers: raw
    pulse widths per channel, held between frames. Normalization is left to
    <code>Vehicles.Interfaces.centeredPwmToUnit</code> and
    <code>Vehicles.Interfaces.throttlePwmToUnit</code> and mode decoding to
    <code>Vehicles.Rdd2.FlightModeSelector</code>, so a closed-loop test
    exercises the same conversion chain a flight would.</p>
    <p>Channel order is {roll, pitch, yaw, throttle, mode switch}. Roll
    positive commands motion to the right and pitch positive commands motion
    forward, the same sense those sticks have in attitude mode.</p>
  </html>"));
end ScriptedTransmitter;
