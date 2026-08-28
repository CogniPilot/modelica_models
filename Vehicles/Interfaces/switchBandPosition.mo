within Vehicles.Interfaces;
function switchBandPosition
  "Quantize a switch channel into a band index with edge hysteresis"
  input Real pwm_us "Raw switch channel, microseconds";
  input Real edge_us[:] "Strictly ascending band edges, microseconds";
  input Real hysteresis_us(min = 0.0) "Half-width of each ambiguous zone";
  input Integer previous(min = 1) "Band index held on the previous sample";
  output Integer position(min = 1) "One-based band index";
protected
  Integer raised "Bands whose raised edge the channel has cleared";
  Integer lowered "Bands whose lowered edge the channel has cleared";
algorithm
  // A pair of ordinary counting quantizers on the raised and lowered edge
  // sets. Away from every edge they agree and the previous index does not
  // matter; inside an ambiguous zone of width 2 * hysteresis_us they straddle
  // the edge and the previous index is retained. That is a multi-level Schmitt
  // trigger written without a branch per band, so it stays correct for any
  // number of switch positions.
  raised := 0;
  lowered := 0;
  for index in 1:size(edge_us, 1) loop
    if pwm_us >= edge_us[index] + hysteresis_us then
      raised := raised + 1;
    end if;
    if pwm_us >= edge_us[index] - hysteresis_us then
      lowered := lowered + 1;
    end if;
  end for;
  position := min(max(previous, raised + 1), lowered + 1);
  annotation(Documentation(info = "<html>
    <p>Transmitter mode switches are read as absolute pulse widths, the way
    ArduPilot reads <code>FLTMODE_CH</code>
    (<code>RC_Channel::read_6pos_switch</code>) and Betaflight reads its
    auxiliary range steps. Neither project applies amplitude hysteresis; both
    rely on a time debounce instead, which rejects glitch frames but cannot
    stop a channel resting on a band edge from chattering. The two mechanisms
    are orthogonal, so this function supplies the amplitude half and
    <code>Vehicles.Rdd2.FlightModeSelector</code> supplies the debounce.</p>
    <p>Correctness requires <code>hysteresis_us</code> below half the narrowest
    band, which the selector asserts. The result is monotone in
    <code>pwm_us</code> and idempotent: feeding the result back as
    <code>previous</code> at an unchanged input reproduces it.</p>
  </html>"));
end switchBandPosition;
