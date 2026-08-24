# ESKF measurement-Jacobian synthesis proof

This directory shows that the rumoca `jacobian` construct synthesizes the
measurement Jacobians of the aided strapdown ESKF in
`Estimation/StrapdownINS/ESKF`, and that the synthesized matrices agree with the
hand-written filter Jacobians on flight math.

## What is proved

Each correction in the ESKF builds its measurement Jacobian in two factors,

    H = delayedH * currentToDelayed

where `delayedH` is the measurement Jacobian at the aiding-delayed state and
`currentToDelayed` transports the tangent across the aiding delay. At zero
aiding delay `currentToDelayed` is the identity (`discreteTransition` with
`dt = 0`), so `H = delayedH`. This proof synthesizes `delayedH`, the
measurement Jacobian itself, for three corrections.

The measurement is written as a Modelica function `h(state)` returning the
predicted measurement, and its tangent-space Jacobian is

    H := jacobian(h(retract(stateBar, delta)), delta)   at delta = 0

`retract` is the ESKF error retraction: right perturbation on SE_2(3),
`inject(nominal, delta) = nominal * exp_map(delta)` in the extended pose
{position, velocity, attitude} with additive gyroscope and accelerometer bias.
This is `Estimation.StrapdownINS.ESKF.inject`. `jacobian` differentiates in the
coordinates of `delta`; `retract` supplies the manifold structure, so `H`
comes out in the filter's 15-element tangent basis
{position, velocity, attitude, gyro bias, accel bias}.

| Correction        | `h(state)`                              | hand `delayedH`                        |
|-------------------|-----------------------------------------|----------------------------------------|
| `correctBarometer`| world-up position `p_z`                 | `[R[3,:]  0]`                          |
| `correctGpsPosition` | body-frame position `R_bar' p`       | `[I_3  0]`                             |
| `correctMagnetometer` | 3-2-1 yaw of the attitude           | `[0 0 0 0 0 0  0  sin(phi)/cos(theta)  cos(phi)/cos(theta)  0 ...]` |

The synthesized `H` recovers each hand `delayedH` from the retraction alone.
For the barometer the third row of the rotation matrix falls out of the SE_2(3)
position retraction; for GPS the body-frame frame rotation collapses the
position block to the identity; for the magnetometer the yaw-rate row falls out
of the quaternion product and the yaw extraction.

## Models

`EskfJacobianProof.mo` is a self-contained package with the retraction and the
three proof models `Barometer`, `GpsPosition`, `MagnetometerYaw`. Each model
computes:

  - `Hsynth` from the `jacobian` construct,
  - `Hhand` from the shipped `LieGroups` library (`SO3.Quat.to_DCM`,
    `SO3.EulerB321.from_Quat`) so the comparison is a parity check against the
    real flight code, and
  - `Jfd`, central finite differences of `h` through `retract`, as an
    independent oracle,

and reports `maxDiffHand = max|Hsynth - Hhand|` and
`maxDiffFd = max|Hsynth - Jfd|`.

The differentiated retraction is transcribed from `LieGroups` with the
cross-package qualified calls rewritten as unqualified siblings, because the
`jacobian` engine descends only through unqualified functions in the call
site's lexical scope. Each body is verbatim except the SO(3) left-Jacobian
guard `sqrt(max(theta_sq, eps))`, written `sqrt(theta_sq)`; that branch runs
only when `theta_sq >= eps`, where the guard is a no-op, and it is unreached at
`delta = 0`. The transcription is validated by the numeric parity itself:
`Hhand` uses the shipped library and the gap is at the floating-point floor.

`EskfJacobianProofFlat.mo` inlines the barometer retraction into one function
with no nested calls; it is used for the direct rumoca-binary evaluation below.

## Running

    export LD_LIBRARY_PATH=...            # gcc + systemd runtime libs
    RUMOCA=/path/to/rumoca
    OMC=/path/to/omc

Inspect a synthesized Jacobian as portable Modelica:

    $RUMOCA compile EskfJacobianProof.mo --model EskfJacobianProof.Barometer \
        --source-root <repo root> --emit-standard-modelica -o BaroStd.mo

Evaluate and elaborate under OpenModelica:

    $OMC <<'EOF'
    setModelicaPath("<repo root>"); loadModel(LieGroups);
    loadFile("BaroStd.mo");
    checkModel(EskfJacobianProof.Barometer);
    simulate(EskfJacobianProof.Barometer, stopTime=1.0, numberOfIntervals=2);
    EOF

Randomized parity sweep over `states.txt` (both gaps, every state and
correction), under OpenModelica:

    ./sweep.sh

Direct rumoca-binary check of the synthesized barometer H over the same states:

    ./flat_sweep.sh

## Results

Representative state, all three corrections elaborate under OpenModelica
(`checkModel` completes successfully). Randomized sweep over 12 states:

| Correction        | max\|Hsynth - Hhand\| | max\|Hsynth - Jfd\| |
|-------------------|-----------------------|---------------------|
| `Barometer`       | 0                     | 3.5e-9              |
| `GpsPosition`     | 5.6e-16               | 7.1e-9              |
| `MagnetometerYaw` | 1.3e-15               | 2.6e-10             |

The hand gap is at the floating-point floor: the barometer H is bit-identical
to `R[3,:]`, and the GPS and magnetometer gaps are round-off of `R' R = I` and
of the yaw-rate trigonometric identity. The finite-difference gap is at the
central-difference floor for the `h = 1e-6` step.

Direct rumoca-binary evaluation of the flattened barometer over the 12 states
gives `max|Hsynth - Hhand| = 0`: the rumoca Solve IR itself confirms the
synthesized H equals the world-up body axis.

## Size and the closed-form rules

The synthesized Jacobian is dense forward mode. Each `jacobian` construct
expands to one tangent function per primitive in the retraction call graph
(twelve `_ad_tangent` functions) plus a wrapper that seeds one tangent sweep
per input coordinate, so a 15-column Jacobian costs 15 sweeps regardless of the
number of rows. No `LieGroups` closed-form derivative rule
(`exp_map_jacobian`, `left_jacobian_exact`, `exp_mixed_*`) is referenced in any
emitted expansion: the construct descends into function bodies rather than
consuming the companion rules. The result is correct but larger than the
closed-form chain, which is the honest efficiency baseline for a comparison
against a closed-form Jacobian generator.

## Frontier

  - This proof establishes `delayedH`, the measurement Jacobian. The delay
    transport `currentToDelayed` is a retrodiction transition Jacobian, a
    different object, and is out of scope here. At zero aiding delay it is the
    identity and the full filter `H` equals `delayedH`.

  - The `jacobian` engine differentiates an unqualified call graph. It refuses a
    cross-package qualified callee, so the shipped `LieGroups` library cannot be
    differentiated as imported; the retraction is transcribed with unqualified
    siblings. It also refuses `max`/`min` inside a differentiated body,
    `transpose` of a user-call result whose rank it cannot state, and an
    array-slice actual passed to a formal at a loop-indexed call site.

  - The rumoca Solve IR declines the full nested-retraction expansion in-process
    (`EL005`, "pure-call owner was not issued by this table"), a documented
    limitation for a function that calls two others, is called from a third, and
    returns an array. The expansion is therefore evaluated under OpenModelica,
    which is the same route the compiler's own finite-difference battery takes.
    A flattened single-function retraction is accepted by the Solve IR directly,
    as `EskfJacobianProofFlat` shows.
