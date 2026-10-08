The existing heading-only update is not a general ESKF improvement
================================================================

This development ablation runs all four ESKF horizon/retrodiction and joint/
nonjoint pressure variants on all eight already-inspected captures and all
GPS/denied/loss-return scenarios. It contains 96 new heading-only replays,
compared with the 96 frozen vector-fusion ESKF reference replays. These data
are now development cases, not an untouched validation set for later changes.

The four vector control binaries were rebuilt first and reproduce the frozen
binary hashes exactly. Each heading variant differs only by removing the
EQUIVARIANT_MAGNETOMETER compile definition. Generated Rumoca 0.10.2 objects,
all noise settings, geometric alignment, pressure policy and delayed-data
handling are unchanged. No native filter was modified or rerun in this study.
Existing native results remain in the matched-campaign review.

declaration.json records the complete design before the heading replays.
build.json records all definitions/binary hashes; campaign.json binds every
case. cases/ retains every complete result, its baseline-pilot binding, RMS,
NEES, transition and timing statistics. comparison.csv retains all 576 rows:
both policies, four ESKF variants, eight captures, three scenarios and three
scoring windows. comparison.txt gives complete flight medians and paired win
counts. summary.json gives per-window/per-metric differences and worst losses.

All 96 new state replays complete with full finite/valid state coverage and
valid common 15D covariance in every scored window. Four rebuilt vector binary
controls match exactly. Heading-only does improve some horizontal RMS values;
those results remain in the tables. It nevertheless makes velocity RMS worse
in ALL GPS and denied comparisons for ALL four variants (64/64 pairs), and
makes almost all attitude/yaw pairs worse. It is rejected as a replacement
for calibrated vector fusion, rather than selected on favorable position rows.

Examples, median denied flight RMS across the eight captures:
  variant                 H vector / heading m    yaw vector / heading deg
  horizon                      .37485 / .40181           .33338 / .40183
  retrodiction                 .37698 / .41235           .33059 / .39991
  joint horizon                .35891 / .36843           .33640 / .41586
  joint retrodiction           .35980 / .38144           .33214 / .40794

Joint horizon heading wins horizontal RMS in 6/8 individual denied pairs but
has a worse median than joint horizon vector; the magnitudes of its losses
matter. A count of favorable cases is insufficient to select a default.
Higher mean NEES after the change is also not an accuracy improvement.

Reproduce by using the same frozen production objects and replay bridge from
../common-sensor-noise/replay-build.json. First rebuild all vector definitions
and require their four recorded binary hashes. Then remove only
-DEQUIVARIANT_MAGNETOMETER and replay the same eight capture/arrival inputs
with compare_exposure.eskf, FOH, --stationary-until 120, --mission-offset 107,
--measurement-noise 1e-6 .075 and the original noise-density/delay settings.
The unchanged generated objects and explicit controls distinguish this from
an exporter/compiler/parameter change. Store builds and raw CSVs under
$HOME/scratch. No Modelica algorithm or vehicle default changed.

The denied-flight yaw/position gap remains. Next investigations should retain
vector fusion and examine its noise-dependent symmetric linearization and
flow/tilt/bias coupling, with actual ESKF per-sensor NIS before selecting a
change. Future tuned changes require fresh validation captures. Existing
conditional Lie/covariance proofs do not imply nonlinear firmware superiority.
