#!/usr/bin/env bash
# Fusion-horizon measurement: ARM -Os objdump + host gcov coverage, multiplied
# through dyninl.pl. Usage: bash measure_horizon.sh <warm> <shifted> <tag>
set -euo pipefail
export PATH=/nix/store/sffq2m1v9g60ff8prg579dmmcyryrkam-rust-nightly-2026-02-27/bin:/nix/store/xcnqqnhw9hb4j5rjgds2yjryi8qki5f3-gcc-wrapper-15.2.0/bin:/nix/store/aqfd5hm2v6fqlhbvmf8nqnwl5qwg8xy6-gcc-arm-embedded-15.2.rel1/bin:/nix/store/sq0nrnfwhkc5ljvklnrk5ps4358g4nbj-gcc-15.2.0/bin:$PATH
export LD_LIBRARY_PATH=/nix/store/y84phxg04lf0pv6p3gb78mqr57i4lywc-devenv-profile/lib:/nix/store/vv5bna641lxwxm0nqgy20134y7wivsvp-systemd-260.2/lib:/nix/store/n35z8vvlr7c5k1406n5bwd0f8h2hgj1j-gcc-15.2.0-lib/lib
# Working root. Every path below hangs off it, so the rig is not tied to the
# session it was first run in.
SP=${HORIZON_WCET_ROOT:?set HORIZON_WCET_ROOT to a scratch directory}
GEN=${HORIZON_WCET_GEN:-$SP/gen}/Estimation_FusionHorizon_OutputPredictor/ProductionCode
RIG=${HORIZON_WCET_INSTRUMENTS:?set HORIZON_WCET_INSTRUMENTS to the directory holding dyninl.pl and dyn_asshipped.pl}
HOSTGCC=/nix/store/xcnqqnhw9hb4j5rjgds2yjryi8qki5f3-gcc-wrapper-15.2.0/bin/gcc
HOSTGCOV=/nix/store/sq0nrnfwhkc5ljvklnrk5ps4358g4nbj-gcc-15.2.0/bin/gcov
ARMBIN=/nix/store/aqfd5hm2v6fqlhbvmf8nqnwl5qwg8xy6-gcc-arm-embedded-15.2.rel1/bin
ARMFLAGS="-Os -std=c99 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fno-math-errno"
BASE=Estimation_FusionHorizon_OutputPredictor
WARM=${1:-200}; SHIFT=${2:-0}; TAG=${3:-run}
OUT=$SP/$TAG
rm -rf "$OUT"; mkdir -p "$OUT"; cd "$OUT"

# ---- ARM -Os objects, both translation units. Counting only the model TU
# ---- undercounts: the contraction split put kernels in the second one.
$ARMBIN/arm-none-eabi-gcc $ARMFLAGS -g -fstack-usage -I"$GEN" -c "$GEN/$BASE.c" -o model.o
$ARMBIN/arm-none-eabi-gcc $ARMFLAGS -g -fstack-usage -I"$GEN" -c "$GEN/rumoca_galec_kernels.c" -o kernels.o
$ARMBIN/arm-none-eabi-objdump -dlr --inlines --no-show-raw-insn model.o   > model.disi
$ARMBIN/arm-none-eabi-objdump -dlr --inlines --no-show-raw-insn kernels.o > kernels.disi
$ARMBIN/arm-none-eabi-objdump -dlr --no-show-raw-insn model.o   > model.dislr
$ARMBIN/arm-none-eabi-objdump -dlr --no-show-raw-insn kernels.o > kernels.dislr
$ARMBIN/arm-none-eabi-size -A model.o kernels.o > size.txt

# ---- host coverage build; each TU compiled separately so gcov finds the gcno
mkdir -p cov && cd cov
for s in "${HORIZON_WCET_DRIVER:-$SP/driver_horizon.c}" "$GEN/$BASE.c" "$GEN/rumoca_galec_kernels.c"; do
  $HOSTGCC -Os -g --coverage -std=c99 -fno-math-errno -I"$GEN" -c "$s" -o "$(basename "$s" .c).o"
done
$HOSTGCC -Os -g --coverage ./*.o -o run -lm
./run "$WARM" "$SHIFT" > out.txt
$HOSTGCOV -o . ./*.gcno > gcov.log 2>&1 || true
cd ..

echo "=== driver output ==="; cat cov/out.txt
echo "=== ARM text ==="; cat size.txt | grep -E "\.text|file format"
echo "=== dyninl.pl (all three counting bugs fixed) ==="
perl "$RIG/dyninl.pl" model.disi "$BASE.c" "cov/$BASE.c.gcov" \
                      kernels.disi rumoca_galec_kernels.c cov/rumoca_galec_kernels.c.gcov \
  | tee dyninl.txt | grep -E "^TOTAL|^symbol|FLOP"
echo "=== dyn.pl as shipped, published alongside so the disagreement stays visible ==="
perl "$RIG/dyn_asshipped.pl" \
     model.dislr "$BASE.c" "cov/$BASE.c.gcov" \
     kernels.dislr rumoca_galec_kernels.c cov/rumoca_galec_kernels.c.gcov \
  | tee dyn_asshipped.txt | grep -E "^TOTAL" || true
