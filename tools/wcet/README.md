# Fusion-horizon timing rig

Two instruments, because on this artifact one of them could not be trusted on
its own. `measure_horizon.sh` cross-compiles both translation units for the
Cortex-M7 target and multiplies per-line ARM instruction counts by host coverage
counts. `callgrind_horizon.sh` counts executed instructions exactly on the host
and differences two runs that differ by exactly one tick, which has no per-line
model and no inline weighting. The measurement record explains where they
disagree and by how much.

Neither script hardcodes a path. Set:

    HORIZON_WCET_ROOT          scratch directory the run writes into
    HORIZON_WCET_GEN           generated eFMU ProductionCode directory
    HORIZON_WCET_INSTRUMENTS   directory holding dyninl.pl and dyn_asshipped.pl
    HORIZON_WCET_DRIVER        driver_horizon.c, defaults to $HORIZON_WCET_ROOT

Usage, and the cadence arithmetic that decides what is being measured: a release
boundary falls on ticks 8k+1 and the horizon is ready once the ring holds a full
span, so `400` puts the measured tick ON a boundary and `401` puts it between
boundaries, with the buffer at steady state either way. The second argument
raises the horizon state-shifted flag for the measured tick alone.

    bash measure_horizon.sh   401 0 common-tick
    bash measure_horizon.sh   400 0 release-tick
    bash measure_horizon.sh   401 1 rebase-tick
    bash callgrind_horizon.sh

Both translation units are compiled and counted. Counting only the model TU
undercounts, because the contraction split moved kernels into the second one.
