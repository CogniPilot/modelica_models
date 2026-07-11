# modelica_models

Reusable Modelica building blocks for rigid-body simulation, estimation,
control, and verification.

## Layout

- `Estimation/`: structured estimator prediction and correction functions.
- `LinearAlgebra/`: dimension-generic matrix algorithms used by estimation
  and control code.
- `Polynomials/`: dimension-generic Hermite construction, derivative
  evaluation, and integrated derivative-cost matrices.
- `LieGroups/`: Lie groups including SO(2), SO(3), SE(2), SE(3), and SE_2(3),
  with replaceable quaternion, DCM, MRP, and Euler rotation representations.
  All 12 axis sequences are available in both body-fixed and space-fixed form,
  including `B232` and `S123`.
- `Geodesy/`: reusable local-frame and geodetic conversion helpers.
- `RigidBody/`: reusable six-degree-of-freedom rigid-body dynamics.
- `Planning/`: forward-only bounded-curvature path planning, including all six
  classical Dubins path families.
- `RigidBody/Examples/`: reusable base plants for quadrotor, rover, and
  fixed-wing use cases. These plants contain vehicle-generic dynamics only.

## Testing

All mathematical test definitions are Modelica classes under `Tests/`. The
suite uses Modelica `assert` equations for Lie-group identities, generic
linear-algebra systems, IEKF invariants, Dubins paths, and analytic rigid-body
trajectories.
No Python source generation or numerical oracle is involved.

Run the complete suite in the pinned development environment:

```bash
nix run .#ci
```

The cross-platform Python task runner uses the standard library for process and
filesystem orchestration and Matplotlib only for rendering the PNG figures.
With Python, Matplotlib, and the required compiler tools available, the
equivalent local commands are:

```text
python -m tools.ci test
python -m tools.ci omc
python -m tools.ci rumoca
python -m tools.ci plots
```

The runner uses the pinned OpenModelica container when Docker is operational,
then falls back to a local `omc`. It handles paths, temporary directories,
process failures, and artifact validation without platform-specific scripts.
Python does not compute test oracles: mathematical pass/fail checks remain
Modelica assertions executed by OpenModelica and, where currently supported,
Rumoca.

The CI command also simulates reproducible planning examples and writes CSV
data plus PNG plots under `artifacts/planning/`. The primary aircraft figure is
a closed figure eight whose two crossing headings share exactly one center
position. Redundant identical poses are coalesced, leaving seven waypoint
occurrences and six nonzero Dubins legs. The initial flight plan is overlaid with
its flyable, continuous-curvature Dubins-polynomial smoothing. A separate plot
checks curvature and curvature-derivative continuity across every leg and
waypoint junction, and a mission-wide signed bank-angle plot shows the
coordinated left/right command. Generated artifacts are ignored by Git and
uploaded by the GitHub Actions workflow.

The Dubins-polynomial coefficients are obtained with a fixed-cost analytical
procedure: one joint unconstrained quadratic solve over the free junction
derivatives, followed by one closed-form true-spatial-derivative repair pass.
The default derivative weights are `{0.3, 1, 0, 50}` through third order. The
CI gallery also renders four paper-style comparisons with the Dubins reference,
optimized path, and nominal segment junctions on common axes.

The plotted example also checks a stated aircraft flight envelope in Modelica:
minimum turn radius, coordinated bank angle, and coordinated bank rate at the
declared flight speed. Thus "flyable" means that those explicit assumptions
pass assertions; it is not inferred from geometric smoothness alone.

To regenerate only the planning artifacts:

```bash
nix run .#ci -- plots
```

CI uses Nix to pin both compiler checks. OpenModelica runs the complete
assertion simulation using the pinned container image, with a local `omc` as
the fallback when Docker is unavailable. Rumoca independently parses, resolves, flattens, and lowers
the same `Tests.All` model to DAE form, then executes SO(2) and SE(2) assertion
smoke models with its simulation backend. The complete assertion model remains
the OpenModelica simulation authority because Rumoca does not yet structurally
lower every aggregate static assertion and array-valued dynamic equation.

## Replaceable rotation representations

`SE3.Generic` and `SE23.Generic` accept any package implementing
`SO3.Interfaces.PartialRotation`. Concrete quaternion, MRP, DCM, and Euler-B321
packages are provided. A specific Euler sequence can be selected directly:

```modelica
package VehicleState = LieGroups.SE23.Generic(
  redeclare package Rotation = LieGroups.SO3.EulerSequences.B232);
```

Quaternion and DCM representations are generally preferable for global
simulation. Euler representations are useful at interfaces where a named
sequence is required. Proofs using local log coordinates should explicitly
establish the chart-domain guard supplied by `LieGroups.Analysis`.

## Verification-oriented APIs

The library keeps simulation equations and proof expressions aligned:

- `RigidBody.stateDerivative` is the pure vector field used by
  `RigidBody6DOF`.
- `RigidBody.mechanicalEnergy`, `wrenchPower`, and `powerBalanceResidual`
  expose the lossless passivity identity.
- `LieGroups.Analysis` supplies coordinate-independent attitude potential,
  principal-log domain checks, and dimension-generic tangent-space quadratic
  candidates.
- `LinearAlgebra.contractionResidual`, `quadraticForm`, and
  `quadraticFormDerivative` provide dimension-generic certificate expressions.
- The estimator exposes its nominal discrete map, tangent transition, process
  covariance, measurement map, measurement covariance, and invariant residual
  as separate pure functions for reachable-set propagation.

These functions produce certificate quantities; they do not claim a proof by
themselves. Bounds on chart validity, disturbances, numerical integration, and
the sign of the resulting matrix inequalities remain explicit proof
obligations.

## Dubins path planning

`Planning.Dubins` computes the globally shortest member of the six classical
forward-only, constant-turn-radius path families: LSL, RSR, LSR, RSL, RLR, and
LRL. Positions are Cartesian x/y coordinates and headings are in radians.

The classical `plan` search includes RLR/LRL by default and accepts
`allowThreeTurnPaths=false` when those reversal-heavy paths are inappropriate.
Aircraft trajectory examples disable them by default and automatically select
the shortest feasible LSL, RSR, LSR, or RSL path, so callers do not select a
family manually.

```modelica
Planning.Dubins.Path path;
Planning.Dubins.Pose midpoint;
equation
  path = Planning.Dubins.plan(
    {0.0, 0.0}, 0.0,
    {10.0, 10.0}, 0.5 * 2.0 * asin(1.0),
    5.0);
  midpoint = Planning.Dubins.evaluate(path, 0.5);
```

The path record exposes its selected family, physical length, turn radius, and
three normalized segment lengths. Turn segment lengths are angles in radians;
straight segment lengths are distance divided by turn radius. This makes the
piecewise curvature and segment domains explicit for tracking and reachability
analysis.

### Smooth transverse-polynomial paths

`Planning.DubinsPolynomial` represents a smooth trajectory as a piecewise
polynomial left-normal offset from an underlying Dubins path. Polynomial
abscissas use physical nominal path distance rather than normalized segment
coordinates, keeping derivative costs dimensionally consistent between arcs
and straight segments.

The evaluator accepts any coefficient count and returns:

- position and heading;
- true unit-speed spatial derivatives through third order;
- signed curvature and curvature derivative;
- transverse offset and the offset-to-nominal metric scale.

`smoothOffsets` constructs a septic seed that fixes position and heading at
the endpoints and segment junctions while making curvature and curvature rate
continuous. `derivativeCost` and `Polynomials.derivativeCostMatrix` provide the
quadratic objectives needed to penalize offset, curvature-related derivatives,
and higher derivatives. The coordinated-flight helpers map curvature and its
metric derivative to fixed-speed roll angle and roll rate.

The supplied smoothing seed constrains transverse offset and its first
derivative to zero at nominal segment junctions. This makes its C3 continuity
exact. An optimizer that permits nonzero junction offsets must additionally
apply a nonlinear true-derivative repair iteration; use
`junctionContinuityResidual` as the acceptance certificate. Every candidate
must also retain a positive `metricScale`, since a zero value indicates a
singular or locally reversed offset curve.
