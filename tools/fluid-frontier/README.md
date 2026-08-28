# The MSL.Fluid frontier

**3 of 115 Fluid frontier models compile today**, measured against Rumoca
`d4d80fbb` on Modelica Standard Library 4.1.0.

This directory is the ratchet for the Fluid P0 campaign. `fluid_frontier.py`
compiles a pinned set of MSL models one at a time and writes `frontier.csv`,
one row per model: what happened, which phase it died in, and which
architecture gap the failure evidences. Each landing that moves the compiler
should move the headline number, and nothing should silently move it back.

It is **not** run by CI. It starts up to 115 compiler processes against a full
standard library and costs minutes. Run it by hand when a branch claims to have
moved the frontier, and diff its CSV against the committed one.

## Running it

    python3 tools/fluid-frontier/fluid_frontier.py \
        --rumoca /path/to/rumoca \
        --msl-root /path/to/ModelicaStandardLibrary-4.1.0 \
        --out tools/fluid-frontier/frontier.csv \
        --logs /tmp/fluid-frontier-logs

`--msl-root` wants the **release layout**: the directory holding
`Modelica 4.1.0/package.mo`, `ModelicaServices 4.1.0/` and `Complex.mo`. This
is the same root `tools/rumoca_acceptance.py --msl-root` wants and the same one
`cargo xtask verify corpus-pin` resolves. A bare `Modelica/` checkout is not
that layout and fails every row identically, which is a rig error and not a
measurement.

`--reclassify --logs <dir>` re-derives the buckets from a finished run's logs
without starting a compiler. Every row's **outcome** is carried over unchanged:
what the compiler did is the compiler's to say, and a reclassification must
never be able to turn a failure into a pass. Only the bucket, the evidence and
the mechanism -- the parts this rig authors -- are re-derived.

`--enumerate` regenerates `targets.tsv` from the MSL root. Regenerating it
against a different MSL version changes what the headline number counts, so it
is a deliberate, reviewable act rather than something the measurement does on
its own.

## Provenance of this measurement

| | |
| --- | --- |
| compiler | Rumoca `d4d80fbb5d6c` (`rumoca build-info`), the models CI pin |
| library | MSL 4.1.0 release zip, sha256 `5409cfa17797c52d866dc9d2675badc1378b4bbf88066e60899a489cb1cb8a2d` |
| invocation | `rumoca compile "<msl>/Modelica 4.1.0/package.mo" --model <M> --source-root <msl> --cache-dir <cache>` |
| timeout | 120 s per model |
| targets | `targets.tsv`, 115 models |

The invocation matches the one `infra/verification/corpus-pin.json` uses for
its own MSL rows, entry point and all, so a frontier row and a corpus-pin row
mean the same thing about the same compiler. Front-end depth is the whole
point here, so no target or emission flag is passed: a row is OK when the
compiler produces a balanced DAE.

## The target set

Every **model** (not package, not function, not partial) in:

| package | models | what it tests |
| --- | ---: | --- |
| `Modelica.Fluid.Examples` | 32 | the executable frontier: what Dymola and OMC ship as Fluid tests |
| `Modelica.Fluid.Vessels` | 4 | |
| `Modelica.Fluid.Machines` | 6 | |
| `Modelica.Fluid.Pipes` | 9 | public component models, each instantiable stand-alone |
| `Modelica.Fluid.Valves` | 6 | |
| `Modelica.Fluid.Sources` | 5 | |
| `Modelica.Fluid.Sensors` | 18 | |
| `Modelica.Fluid.Fittings` | 10 | |
| `Modelica.Media.Examples` | 25 | the Media ladder, including the four the brief named: `SimpleLiquidWater`, `IdealGasH2O`, `WaterIF97`, `MoistAir` |
| **total** | **115** | |

## The scoreboard

| package | total | compiling | p0a-specialization | zero-extent | genuinely-new | declared-unsupported |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `Fluid.Examples` | 32 | 1 | 23 | 5 | 3 | 0 |
| `Fluid.Fittings` | 10 | 0 | 9 | 1 | 0 | 0 |
| `Fluid.Machines` | 6 | 1 | 5 | 0 | 0 | 0 |
| `Fluid.Pipes` | 9 | 0 | 9 | 0 | 0 | 0 |
| `Fluid.Sensors` | 18 | 0 | 18 | 0 | 0 | 0 |
| `Fluid.Sources` | 5 | 0 | 5 | 0 | 0 | 0 |
| `Fluid.Valves` | 6 | 0 | 6 | 0 | 0 | 0 |
| `Fluid.Vessels` | 4 | 0 | 4 | 0 | 0 | 0 |
| `Media.Examples` | 25 | 1 | 18 | 0 | 5 | 1 |
| **total** | **115** | **3** | **97** | **6** | **8** | **1** |

The three that compile are
`Modelica.Fluid.Examples.HeatExchanger.BaseClasses.WallConstProps`,
`Modelica.Fluid.Machines.BaseClasses.PumpMonitoring.PumpMonitoringBase` and
`Modelica.Media.Examples.Utilities.ShortPipe`. None of them touches a medium
property: the first is a metal wall, the second a monitoring base class with an
empty medium interface, the third a linear pressure drop.

**No row is a timeout, a panic, an internal error or a signal.** All 112
failures are typed refusals with a diagnostic code, and none of the 115 models
came close to the 120 s ceiling. Those rows would have their own lines in this
table if they existed; they do not, and that is a real and creditable property
of the pin.

Where each failure dies:

| phase | models | codes |
| --- | ---: | --- |
| typecheck | 51 | `ET001` 38, `ET002` 13 |
| instantiate | 30 | `EI012` 25, `EI007` 3, `EI027` 2 |
| flatten | 21 | `EF019` 9, `EF024` 8, `EF025` 2, `EF015` 1, `EF016` 1 |
| dae | 10 | `ED008` 4, `ED010` 3, `ED019` 3 |

## Findings: which P0 lands first

**P0-A effective specialization, by a margin that leaves nothing to argue
about. It blocks 97 of the 112 failures, and the remaining 15 are behind it.**

Everything else in the taxonomy scores **zero**, and the reason is not that
those gaps are absent. It is that **no model in the corpus survives long enough
to reach them**:

| bucket | blocked | why the number is what it is |
| --- | ---: | --- |
| p0a-specialization | 97 | measured |
| genuinely-new | 8 | measured; see below |
| zero-extent | 6 | measured, but it is a P0-A interaction (below) |
| declared-unsupported | 1 | MLS §12.4.2.1 partial application, honestly refused |
| p0c-topology | 0 | **masked.** Connection processing runs in flatten; 81 of 112 failures die in typecheck or instantiate, before any connect set is built |
| p0d-stream | 0 | **masked.** Not one `inStream`/`actualStream` diagnostic in 115 models. The stream lowering the tranche convicted is never reached |
| p0e-balance | 0 | **masked.** Balance is a late global count; nothing reaches it |
| as051 | 0 | **masked**, and worth stating plainly: the known Boolean-connector defect cannot be observed anywhere on this corpus. Its fix will not move this number |
| p0g-annotations | 0 | **masked.** `derivative`/`inverse`/`smoothOrder` are pervasive in Fluid and Media and are never consulted |

So the campaign's priority order is not a ranking of five comparable gaps. It
is **P0-A, then re-measure**. Any sequencing that puts P0-C, P0-D or P0-E first
would be optimising a phase that no Fluid model currently reaches, and the
scoreboard would not move at all. The honest read of this table is that P0-A is
not merely first, it is the only gap this corpus can currently see.

### The zero-extent dependency is real, and it is a P0-A interaction

Six models fail with `ET001 unknown member m_flow` on a connector array:
five `AST_BatchPlant` models on `topPorts.m_flow`, where `nTopPorts` defaults
to `0`, and `Fittings.MultiPort` on `ports_b.m_flow`, where `nPorts_b` defaults
to `0`. These are **zero-extent connector arrays**, and the member does not
vanish because the array is empty. It vanishes because the
element type draws its members from a replaceable package.
`tools/rumoca-repros/medium-slot-as-type/` shows all three cases in one file: at
extent zero with a package slot the member is reported unknown, at any extent
with a package slot the type collapses to the slot, and the identical
zero-extent array of a connector with no replaceable package compiles.

This is exactly the missed dependency the boundary verdict named: the
checked-empty aggregate path must carry zero-extent members through connectors
and signatures rather than reading absence of scalar leaves as absence of the
member. The measurement says it cannot be designed independently of P0-A.

### Which deletion site each blocked model convicts

The `mechanism` column names the compiler site the evidence points at. Each was
confirmed present in the pinned tree before being cited.

| mechanism | models | site |
| --- | ---: | --- |
| ALIAS | 55 | `rumoca-phase-instantiate/src/type_overrides/override_map.rs` -- an exact alias DefId falls back to a path key, missing declarations are filter-mapped away, and a missing selected package or member returns rather than refusing |
| FIXPOINT | 37 | `rumoca-phase-flatten/src/constant_extraction.rs` -- bounded recovery loops, `MAX_PASSES` 4 and 5 |
| GUESSNAME | 14 | `rumoca-phase-flatten/src/equations/mod.rs::lookup_parameter_in_scope` -- classifies a reference by initial capitalisation, retries case-mangled spellings, then infers `nX`/`nXi`/`nC`/`nS` from already-known array dimensions |
| SIGWALK | 2 | `rumoca-phase-typecheck/src/function_signatures.rs` -- parallel string/DefId/unqualified maps and a depth-64 walk whose overflow, cycle and missing cases continue |
| (none) | 4 | the three `ED010` StateGraph rows and one declared-unsupported row |

The FIXPOINT attribution is not inferred from the file alone. It is measured:
**the largest single failure shape does not reproduce at shallow depth.** The
declaration at `Fluid/Interfaces.mo:1028` that refuses 32 models,

    Medium.MassFlowRate[m] m_flows(each min = ..., each start = ..., each stateSelect = ...)

compiles without complaint in a ten-line stand-alone model that uses the real
`Modelica.Media.Interfaces.PartialMedium`, with the dimension on the type or on
the name, with or without the attribute modifiers, with the medium left at its
partial default or redeclared to `StandardWater`. It fails only inside MSL's
own chain. A failure that appears when structural depth grows and disappears
when it shrinks is a bounded-pass recovery reaching its ceiling, which is the
signature the campaign asked to be watched for.

The same shallow-sibling-passes pattern holds for the three `ED010` StateGraph
rows: neither the `inner`/`outer` discrete propagation nor the array-valued
discrete equation reproduces outside MSL. They are recorded by their MSL site
rather than by a synthetic repro that does not reproduce.

## Genuinely-new failures

Two have minimal repros in `tools/rumoca-repros/`; the third resisted
reduction and is recorded honestly as such.

1. **`package-record-constant/`** -- every member access on a package-level
   constant of **record** type is an unresolved Flat reference (`ED008`),
   however the value is supplied: by a modification on the declaration or by a
   record-constructor binding. A **scalar** constant in the same package
   resolves. Blocks `Media.Examples.IdealGasH2O`,
   `Media.Examples.ReferenceAir.DryAir2` and
   `Media.Examples.ReferenceAir.Inverse_sh_T` at
   `SingleGasNasa.data.name` and `Air_Utilities.Basic.Constants.h_off`, and it
   is the same root as the eight `EF024` rows that lose `molarMass` off
   `fluidConstants[1]`.

2. **`medium-slot-as-type/`** -- a component declared `Medium.Pressure` is
   reported as having the *package slot* `FluidPort.Medium` as its type, and at
   extent zero the member is reported not to exist at all. Blocks the eight
   `Fluid.Sensors` rows, two `IdealHeatTransfer` rows, and the six zero-extent
   rows above: sixteen models between its two faces.

3. **`ED019` structural evaluator, no repro filed** -- two Media rows fail
   evaluating the same `h_min` parameter binding, one with "division by zero"
   and one with "type mismatch: expected comparable, got Integer <
   Enumeration", where the source compares two `Real`s
   (`if T < data.Tlimit`). The operands are being mistyped, not the comparison.
   It did not reduce to a shallow repro; the CSV records the MSL site.

## Honesty notes

- Media does **not** mask Fluid here. Media models fail in the same P0-A way
  Fluid models do (18 of 25), so no Fluid classification is hidden behind a
  Media cascade. Where a Media model fails for its own reason -- the record
  constants, the evaluator, partial application -- it is bucketed on that first
  genuine gap and said so.
- `UNCLASSIFIED` is a supported bucket and this run has none, but only because
  every shape was examined against its diagnostic and its MSL source. It was
  not driven to zero by widening a rule until everything matched: two rules
  were narrowed after their first version over-claimed, and one bucket
  (`p0c-topology`) that an early rule had assigned 18 models was emptied
  entirely once the evidence showed those were function-identity and
  specialization failures that merely carried an `EF` code.
