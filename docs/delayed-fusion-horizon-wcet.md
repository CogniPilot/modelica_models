# Fusion horizon timing on the flight target

Target: NXP MR-VMU-TROPIC, i.MX RT1064, Cortex-M7 at 600 MHz, single precision
throughout. At an 800 Hz inertial tick the whole core has

    600e6 / 800 = 750,000 cycles per tick

for everything it runs. The question this record answers is what fraction of
that the fusion horizon takes.

## Provenance

Recorded so the numbers can be reproduced or refuted rather than believed.

| item | value |
| --- | --- |
| compiler | Rumoca `d4d80fbb5d6c`, the models CI pin, built in a clean detached worktree |
| corpus | `modelica_models` `b743167` plus this branch |
| target flags | `-Os -std=c99 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fno-math-errno`, `arm-none-eabi-gcc` 15.2.rel1 |
| translation units | both: the model TU and `rumoca_galec_kernels.c`. Counting only the first undercounts, because the contraction split moved kernels into the second |
| disassembly | `arm-none-eabi-objdump -dlr --inlines --no-show-raw-insn` |
| instrument A | `dyninl.pl`, md5 `3ed674868c54b9a0548a4328a10a6abd`. Per-line ARM instruction counts multiplied by host coverage counts. An upper bound: `gcc` puts a loop's per-iteration test and its once-per-entry setup on the same source line while coverage reports one count for it |
| instrument B | `valgrind --tool=callgrind` on the host build. Exact executed-instruction counts on x86, differenced between two runs that differ by exactly one tick. No per-line model, no inline weighting |
| method | 400 or 401 warm ticks, coverage counters reset, then exactly ONE tick of the kind under test. The cadence is chosen so the measured tick lands on or off a release boundary as required |
| rig | `tools/wcet/` in this repository |

Instrument B exists because instrument A could not be trusted here. On the
re-base path A reported the composition running about 35 times per fold
iteration where the algorithm calls it once, and A and B disagreed by 2.5x. B
agrees with A to 1.27x on the common tick, which is the ARM-to-x86 ratio the
codegen scoreboard already records for other artifacts, so both are reported
and the disagreement is left visible.

## The table

| what | ARM executed instructions (A) | exact x86 (B) | share of 750,000 at 1 IPC |
| --- | --- | --- | --- |
| one 800 Hz tick: integrate, accumulate, store, compose | 87,554 | 68,999 | 11.7% |
| one 800 Hz tick that also releases a window to the filter | 84,500 | not differenced | 11.3% |
| one re-base: fold the buffer, move the bias, recompose | not trusted | +81,983,049 | 10,931% |
| 100 Hz filter prediction from an accumulated delta, no correction | 304,363 | not differenced | 5.1% of the 6,000,000 cycle budget at 100 Hz |

**What changed after these numbers were taken.** The counts above are from the
block as it stood before the epoch, edge-trigger and supervision fixes. Those
changed the re-base TRIGGER and the re-base ARITHMETIC. A fold now walks one
more iteration -- twenty-three compositions rather than twenty-two, about 4.5
percent -- because the window accumulated since the last adopt rides the same
walk as a trailing row. The common tick gained one square root, three
comparisons and an integer increment.

The trailing row rides the walk rather than being composed at the call site
for a reason this record already explains. Written as two extra calls in
`step.mo`, an unpack and a composition, the production lowering of the block
stopped completing: 37 seconds became more than eight minutes with no result,
which is the same per-component materialization of record-valued calls that
costs the re-base its 82 million instructions, showing up in the compiler
rather than in the generated code. One more iteration of a loop that already
exists costs one composition.

Neither figure is re-measured here; a re-run of `tools/wcet/` is the way to
replace them, not an estimate.

Flash, both translation units, `-Os`: 148,596 B of `.text`.
RAM: `sizeof(State)` 24,012 B, working memory 5,440 B, which is the ring plus
the live window plus the published packet.

At the stated CPI range for the dual-issue M7:

| | CPI 0.7 | CPI 1.0 | CPI 1.2 |
| --- | --- | --- | --- |
| one 800 Hz tick | 61,288 cycles, 8.2% | 87,554, 11.7% | 105,065, 14.0% |
| one re-base | 57.4M cycles | 82.0M | 98.4M |

## The verdict, unsoftened

**The 800 Hz path fits, with about eight times margin. The re-base does not fit,
by two orders of magnitude, and the reason is the code generator rather than the
architecture.**

Between corrections, which is every tick the filter does not accept aiding, the
horizon costs 8 to 14 percent of the whole core's tick budget across the CPI
range. That is the number the design is judged on for the common case and it
passes.

A re-base costs 82 million instructions. The algorithmic content of a re-base is
22 SE_2(3) compositions. Measured on the same binary, one composition is about
1,480 ARM instructions, so the algorithm's cost is about 33,000 instructions:
4.4 percent of the tick budget, comfortably inside it. The measured figure is
about 2,500 times that.

The cause is identified, not guessed. In the generated production code a
record-valued function result is materialized once per component of the record.
A `Delta` carries 56 fields, so a composition written inside a loop is evaluated
about 35 times per iteration. Callgrind counts the fold calling the composition
32,076 times where the algorithm calls it 22 times per fold. Binding every
record-valued call to a local first, which the source now does everywhere, cut
the re-base from 117M to 82M but did not remove the expansion.

Consequently, with today's compiler:

- One accepted correction costs ONE re-base, so the maximum rate at which a
  correction can be applied is 600e6 / 82e6, about **7.3 Hz**. That is below
  the GPS rate, so the horizon as generated today cannot re-base on every GPS
  fix.
- Nothing else about the architecture is implicated. The common tick, the
  release, the buffer, and the composition algebra are all inside budget.

**That number was not true when this record was first written, and the reason
is worth keeping.** The re-base used to be triggered by
`filter.status.correctionOutcome == CorrectionAccepted`, which is a LEVEL: it
stands for the whole 100 Hz filter tick, which is eight inertial ticks. One
accepted correction therefore fired deltasPerFusion full folds, not one, and at
the flight rates that is eight. The real ceiling was 7.3 / 8, about
**0.91 Hz**, which is below the mocap rate and below the barometer rate as well
as below GPS. The adversarial review that found this reports measuring 160
shifted ticks in the 161 following a single correction; that figure is quoted,
not reproduced here, because no simulation in this tree can run the composed
block (see `Tests.HorizonEstimatorWiring`). What IS in the tree is the
arithmetic and the boundary field the fix rests on.

The trigger is now an edge on
`Avionics.EstimatorStatus.acceptedCorrectionCount`, one re-base per accepted
correction, and the 7.3 Hz above is what the design is judged on. A rising edge
on the level would not have been enough: back-to-back accepted corrections hold
the level true across the filter-tick boundary and the second one would never
reach the predictor, which is why the count was added to the boundary rather
than reconstructed downstream.

**The re-base budget now has a second claimant.** The incremental path composes
tick factors integrated at the ANCHOR bias and does not move them to the
filter's bias; only a re-base does that. Between re-bases the predictor
therefore drifts from the filter at `||db_g||`, without bound in the time since
the last one, which under sustained correction rejection is the whole rejection
episode. `OutputPredictor.maximumPredictorDivergence_rad` bounds it by forcing a
fold when the accumulated divergence would exceed it. At the default 1e-3 rad
and a 5e-3 rad/s bias offset that is one forced re-base every 0.2 s, about
5 Hz, and the two claimants add: the correction rate plus the re-anchor rate is
what must stay under 7.3 Hz, not the correction rate alone. Raising the
tolerance trades predictor accuracy for re-base rate and is the dial to reach
for if the code generator is not fixed first.

Three ways out, in the order they should be tried:

1. **Fix the code generator.** Stop materializing a record-valued call per
   component. This is upstream work and the model does not change when it lands.
2. **Take the record out of the fold.** Compose row to row rather than
   `Delta` to `Delta` inside `foldBuffer`, so the loop never carries a record.
   Contained, model-side, and measurable; not implemented here.
3. **Take the fold out of the re-base.** The peel identity
   `D(t1->t2) = D(t0->t1)^-1 (x) D(t0->t2)` reduces a re-base to one inverse and
   one composition. The design document records why the fold was preferred, and
   those reasons stand; this is the fallback the numbers would force, not the
   choice they should drive.

Two smaller costs are worth naming because a reader will otherwise attribute
them to the algebra. The ring store and the ring read are branch-free
fixed-length walks over the whole buffer, so they cost 22 x 56 multiply-adds per
tick each whether or not anything is stored. That is deliberate: it makes the
worst case the measured case, and the alternative is a dynamic array index the
code generator refuses to lower because it cannot prove the bound. And the
first version of this buffer was tick-granular rather than release-granular; it
measured 329,144 instructions per tick, about nine tenths of it moving the
buffer. Release granularity is exact by the same composition lemma and is what
the current numbers are for.

## What the filter side did not cost

The 100 Hz filter step was NOT made cheaper by this work, and the reason is
worth stating rather than claiming a saving. The corpus has fed the estimator an
accumulated 100 Hz preintegral since commit 8e19eba, which is what let the
estimator rate drop from 200 Hz without losing samples. This architecture keeps
that interface exactly; it changes WHERE the packet is fused, not what it
contains. The prediction cost is therefore unchanged, and the saving the delayed
horizon was expected to produce had already been banked.

Measured here for the record: one prediction step with no aiding fresh costs
304,363 executed instructions and 84,633 flop-equivalents, which is 5.1 percent
of the 6,000,000 cycle budget a 100 Hz tick has. That is the number to compare
against when the horizon rewiring lands, because the rewiring changes which
epoch the step runs at and nothing about the step itself. The horizon does not
move the filter onto the 800 Hz tick and must not be read as doing so.
