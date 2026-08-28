within Vehicles.Rdd2;

// SPDX-License-Identifier: Apache-2.0

block FlightModeSelector
  "Decode a transmitter mode switch into the RDD2 flight-mode enumeration"

  parameter Real samplePeriod(unit = "s") = 0.01
    "Receiver service interval";
  parameter Real bandEdge_us[:] = {1200.0, 1800.0}
    "Strictly ascending band edges of the switch channel";
  parameter Real hysteresis_us(min = 0.0) = 25.0
    "Half-width of the ambiguous zone at each band edge";
  parameter Real debounceTime_s(unit = "s", min = 0.0) = 0.2
    "Continuous agreement required before a new band is accepted";
  parameter Real minimumChannel_us = 800.0
    "Below this the channel is a failed link, not a switch position";
  parameter Real maximumChannel_us = 2200.0
    "Above this the channel is a failed link, not a switch position";
  parameter Integer modeForPosition[size(bandEdge_us, 1) + 1] = {1, 3, 2}
    "Flight mode published for each band, low to high";

  final parameter Integer bandCount = size(bandEdge_us, 1) + 1;
  final parameter Integer debounceSamples(min = 1) =
    max(1, integer(ceil(debounceTime_s / samplePeriod)))
    "Samples of agreement equivalent to debounceTime_s";

  input Real switchPwm_us "Raw mode-switch channel, microseconds";
  output Integer mode(start = modeForPosition[1], fixed = true)
    "0=acro, 1=attitude, 2=mission, 3=pilot position";

protected
  discrete Integer position(start = 0, fixed = true)
    "Accepted band index; 0 until the first valid sample seeds it";
  discrete Integer pending(start = 0, fixed = true)
    "Band index currently accumulating agreement";
  discrete Integer agreement(start = 0, fixed = true)
    "Consecutive samples the pending band has been observed";
  discrete Integer candidate(start = 0, fixed = true)
    "Band the hysteretic quantizer reports this sample";
  discrete Boolean channelValid(start = false, fixed = true);

equation
  assert(samplePeriod > 0.0,
    "FlightModeSelector sample period must be positive");
  assert(maximumChannel_us > minimumChannel_us,
    "FlightModeSelector channel validity window is empty");
  for index in 1:size(bandEdge_us, 1) loop
    assert(bandEdge_us[index] > minimumChannel_us
      and bandEdge_us[index] < maximumChannel_us,
      "FlightModeSelector band edge lies outside the validity window");
  end for;
  // Hysteresis wider than half the narrowest band would let two bands claim
  // the same channel value, which makes the accepted position depend on the
  // path taken rather than on the switch.
  for index in 2:size(bandEdge_us, 1) loop
    assert(bandEdge_us[index] - bandEdge_us[index - 1] > 2.0 * hysteresis_us,
      "FlightModeSelector hysteresis is at least half the narrowest band");
  end for;

algorithm
  when sample(0.0, samplePeriod) then
    channelValid := switchPwm_us > minimumChannel_us
      and switchPwm_us < maximumChannel_us;
    candidate := Vehicles.Interfaces.switchBandPosition(
      switchPwm_us, bandEdge_us, hysteresis_us, max(1, pre(position)));

    if not channelValid then
      // A failed link holds the mode the pilot last selected. Falling back to
      // a default here would change flight mode on a single dropped frame.
      position := pre(position);
      pending := pre(position);
      agreement := 0;
    elseif pre(position) == 0 then
      // Seed from the first valid sample without announcing a change, as
      // ArduPilot seeds a switch on its first radio read. The seed uses the
      // memoryless quantizer because there is no previous position yet.
      position := Vehicles.Interfaces.switchBandPosition(
        switchPwm_us, bandEdge_us, 0.0, 1);
      pending := position;
      agreement := 0;
    elseif candidate == pre(position) then
      // Agreement with the held band restarts the debounce, so a switch that
      // bounces back through its current position stays put.
      position := pre(position);
      pending := pre(position);
      agreement := 0;
    elseif candidate == pre(pending) then
      agreement := pre(agreement) + 1;
      position := if agreement >= debounceSamples then candidate
        else pre(position);
      pending := candidate;
    else
      position := pre(position);
      pending := candidate;
      agreement := 1;
    end if;

    mode := modeForPosition[position];
  end when;

  annotation(Documentation(info = "<html>
    <p>This is the RDD2 twin of ArduPilot's <code>FLTMODE_CH</code> decode and
    PX4's <code>RC_MAP_FLTMODE</code> slot mapping: one continuous receiver
    channel, quantized into bands, mapped through a table into the flight-mode
    enumeration the guidance task already consumes.</p>
    <h4>Why both hysteresis and a debounce</h4>
    <p>No production autopilot uses amplitude hysteresis on the mode channel;
    all three surveyed projects use hard thresholds and reject glitches with a
    time filter (ArduPilot 200 ms of continuous agreement, PX4 two identical
    frames, Betaflight none). A time filter is the right answer for impulsive
    noise but does nothing for a switch resting on a band edge, where it merely
    delays each chatter by the debounce interval. Hysteresis is the right
    answer for that case and does nothing for glitch frames. The two are
    orthogonal, so this block applies both:
    <code>Vehicles.Interfaces.switchBandPosition</code> supplies the amplitude
    half and the counter here supplies the temporal half.</p>
    <h4>Mode table</h4>
    <p>The default table maps a three-position switch low to high onto
    attitude, pilot position, and mission, matching the conventional PX4
    Stabilized / Position / Mission ordering. Only modes RDD2 actually
    implements appear; acro remains reachable by configuring
    <code>modeForPosition</code>, because a three-position switch cannot carry
    four modes.</p>
    <h4>Timing</h4>
    <p>All state advances on one <code>sample</code> clock and every history
    reference is an explicit <code>pre</code>, so the decode is a pure
    synchronous function of this sample and the previous accepted state, with
    no reliance on implicit delay.</p>
  </html>"));
end FlightModeSelector;
