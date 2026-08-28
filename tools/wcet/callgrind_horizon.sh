#!/usr/bin/env bash
# Exact dynamic instruction count for ONE tick, by differencing two runs that
# differ by exactly one tick. Host x86 rather than ARM, and therefore a
# cross-check on the line-attribution instrument rather than a replacement for
# it: it counts what was executed, with no per-line model and no inline-weight
# heuristic.
set -euo pipefail
export PATH=/nix/store/xcnqqnhw9hb4j5rjgds2yjryi8qki5f3-gcc-wrapper-15.2.0/bin:/nix/store/l8kvr38dk84afq0bffmsdybfix0wdvci-valgrind-3.26.0/bin:$PATH
# Working root. Every path below hangs off it, so the rig is not tied to the
# session it was first run in.
SP=${HORIZON_WCET_ROOT:?set HORIZON_WCET_ROOT to a scratch directory}
GEN=${HORIZON_WCET_GEN:-$SP/gen}/Estimation_FusionHorizon_OutputPredictor/ProductionCode
BASE=Estimation_FusionHorizon_OutputPredictor
OUT=$SP/callgrind
rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"
# No coverage instrumentation here: the counters would be counted too.
sed 's/extern void __gcov_reset(void);/static void __gcov_reset(void) {}/; s/extern void __gcov_dump(void);/static void __gcov_dump(void) {}/' \
  "${HORIZON_WCET_DRIVER:-$SP/driver_horizon.c}" > driver_plain.c
gcc -Os -g -std=c99 -fno-math-errno -I"$GEN" -c driver_plain.c -o driver.o
gcc -Os -g -std=c99 -fno-math-errno -I"$GEN" -c "$GEN/$BASE.c" -o model.o
gcc -Os -g -std=c99 -fno-math-errno -I"$GEN" -c "$GEN/rumoca_galec_kernels.c" -o kernels.o
gcc -Os -g driver.o model.o kernels.o -o run -lm
count () {  # warm shifted
  valgrind --tool=callgrind --callgrind-out-file=/dev/null ./run "$1" "$2" 2>&1 \
    | sed -n 's/.*refs: *//p' | tr -d ,
}
echo "warm  shifted  totalIr"
for pair in "400 0" "401 0" "408 0" "409 0" "401 1" "400 1"; do
  set -- $pair
  echo "$1 $2 $(count "$1" "$2")"
done
