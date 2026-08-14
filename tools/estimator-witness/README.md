# Estimator witness harnesses

Before/after evidence for the affirmative-acceptance and recovery-ladder
changes. These drive the **generated GALEC C**, not the Modelica, because the
properties at stake are about real NaN and infinity bit patterns and about
binary32 rounding, neither of which the Modelica assertion suite can reach:
`Tests.EstimatorHardeningTests` is evaluated as a folded parameter binding, and
rumoca rejects a literal `0.0/0.0` there (ED019) before any assertion runs. The
Modelica tests pin the same branches using magnitudes past
`FiniteMagnitudeLimit`; these harnesses pin the actual NaN.

## Build

```sh
rumoca compile Vehicles/Rdd2/NavigationEstimator.mo \
  --model Vehicles.Rdd2.NavigationEstimator --source-root . \
  --target galec-production --output /tmp/est
P=/tmp/est/Vehicles_Rdd2_NavigationEstimator/ProductionCode
cc -O2 -DAFTER=1 -I$P tools/estimator-witness/estimator_witness.c $P/*.c -lm -o /tmp/wit
/tmp/wit /tmp/trace.txt
```

Build the same file with `-DAFTER=0` against a pre-change tree to get the
comparison arm. The two arms must agree byte for byte on the nominal trace.

## What each section establishes

- **A. Non-finite inputs, per channel.** Every aiding channel (GPS position,
  GPS velocity, GPS covariance, mocap, optical-flow velocity, optical-flow
  covariance) plus both IMU channels, driven with real `NAN` and `INFINITY`.
  Each must be rejected with a named cause and leave the state finite. Before
  the change all eight were accepted and poisoned the state permanently while
  `estimate.valid` stayed 1.
- **B. Attitude guarantee through stages 1 and 2.** A 2 km consistent GPS
  outlier stream against a vehicle flying straight and level at a known yaw.
  Attitude drift from a truth-locked reference must stay bounded through both
  recovery stages -- this is the property the wrapper's ATTITUDE degradation
  mode depends on. It also checks the state is never teleported onto the
  rejected fix and that `estimate.valid` never drops.
- **D. Commanded reset.** The one path that still re-seeds state from aiding.
  It must move position onto the fix and restore the declared initial
  covariance -- this is the deliberate operator-level recovery the automatic
  ladder deliberately does not perform.
- **C. Nominal 4000-tick trace.** Truth starts at rest exactly where the filter
  initializes and accelerates gently, so no correction is ever gate-rejected
  and no recovery stage is entered. This trace must be `%a`-identical across
  the change. A cold start against an already-moving stream is deliberately NOT
  used here: that is a large-innovation acquisition, which this change alters
  on purpose.

`trust_region_probe.c` is the diagnostic that isolated the large-innovation
attitude failure: it prints per-tick state around the first accepted
correction, and shows the failure reproducing with the recovery ladder
disabled entirely, which is what established that the ladder was not its cause.

## flow_acquisition_matrix.c

Flow-only truthful acquisition over 4/5/6/8 m/s x 10/50/200/1000 Hz aiding.
Optical flow is the only source and it reports the true body velocity, so
attitude is not directly observed and the correct attitude correction is
always zero. This is the case where a correction direction is valid only when
taken in full, and it is what a per-tick rotation limit gets wrong: limiting
the whole tangent vector starves the states the measurement determines, the
residual never collapses, and the attitude walks a fixed step per accepted
sample.

Every cell prints its own verdict and a failing run names each failing
speed-rate cell and exits non-zero, so a regression identifies the case
instead of surfacing as a silent hang.

Measured: the pre-change estimator fails 7 of 16 cells (V=4/1000Hz,
V=5/1000Hz, V=6/200Hz, V=6/1000Hz, V=8/10Hz, V=8/200Hz, V=8/1000Hz); all 16
track once the limit applies to the rotation alone.

## known_limitation_slow_liar.c

Pins a declared limitation rather than a desired behaviour: a GPS lying by
2 km at 1.67 Hz, with healthy 50 Hz optical flow also present, does not fire
the recovery ladder, because the anchor genuinely alternates between two live
sources.

Accepted because the failure the ladder exists to catch is absent: the
solution stays good (errX -0.034 m) and the liar is still isolated by its own
per-source counter (peak gpsConsecutiveRejections 50). Detection without false
recovery. The file asserts those three properties -- solution good, liar
isolated, ladder quiet -- and fails by name if any of them changes, so the
limitation cannot drift silently in either direction.
