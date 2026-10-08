Native PX4 height/terrain policy observation
8 October 2026

Three read-only native replays now observe public height/terrain getters.
Every published state and innovation CSV is byte-identical to the frozen
seed-911 readiness pilot. No PX4 core, parameter, sensor input or arrival trace
changed. This observes a policy difference; it does not isolate its accuracy
effect or explain the whole vertical RMS gap.

All three scenarios use the same 120-166.7 s flight scoring window. Among its
4,670 post-update observations in each scenario:

  range-height aiding active:             3.4904%
  range-terrain aiding active:          100.0000%
  optical-flow terrain aiding active:    96.2955%
  barometer-height aiding active:       100.0000%
  range used as height reference:         3.4904%
  barometer used as height reference:    96.5096%
  range kinematically consistent:       100.0000%
  range fault flag:                       0.0000%

GPS-height aiding is active for 100%, 0% and 67.4518% of GPS, denied and
loss/return observations respectively. These fractions describe mode flags
after update, not the number of accepted sensor corrections or independent
statistical trials. The range/flow terrain flags may both be active.

The adapter configures BARO height reference and conditional range-height
control. Native range_height_control.cpp checks height, horizontal speed and
innovation stability, with hysteresis, before enabling direct range-height
aiding. It separately supports terrain-state correction. Actual mode traces
show that direct range height is only a small part of this capture's flight;
one should not infer continuous range-height fusion from the parameter alone.

Current ESKF correctOpticalFlow uses measured co-timed range to scale angular
flow and propagates range uncertainty into flow covariance. It does not fuse
that range as a separate height/terrain-state observation in this function.
ArduPilot's adapter selects barometer height and sets EK3_RNG_USE_HGT=-1;
native PosVelFusion requires that parameter to be positive for conditional
range-height switching. This static source check does not observe every EKF3
terrain or height-fusion event. Native effective sensor/state models remain
different despite identical physical samples.

A future ESKF range/terrain extension needs an explicit terrain model and
shared measurement correlations. Simply asserting known ground height or
fusing the same noisy range independently in height and flow would change
the comparison assumptions or double-count information. No such extension
or superiority claim is introduced by this diagnostic.

Evidence and reproduction

observer.json binds pilot, adapter, header, core and diagnostic hashes.
*-height.csv retain every raw observation, including preflight. analysis.json
contains the three complete summaries. source-bindings.json records local
source hashes without adding native source to this repository.

replay_px4_magnetics.py accepts --quantity height, retaining magnetic as its
default. A further three native replays verify backward compatibility:
default-magnetic.json matches every frozen published state and innovation,
and its staged adapter and every magnetic trace are byte-identical to the
previous magnetic observer in ../matched-campaign/px4-magnetic-observer.json.
Thus six replays validate both observer paths against the same native core.

Run the observer with the existing readiness pilot and owned native builds:
  python tools/estimator_comparison/replay_px4_magnetics.py --quantity height
    --pilot PILOT.json --cases PILOT_CASES --harness HARNESS
    --px4-source OWNED_PX4 --px4-library LIBEKF2 --cxx CXX
    --work OWNED_FRESH_WORK --output OWNED_FRESH_JSON
Large binaries and replays remain under $HOME/scratch/modelica_models/px4-height.
Recompute this summary without native replay:
  python summarize.py --root THIS_REVIEW --output OWNED_FRESH_JSON
evidence-sha256.json binds the copied artifacts.
