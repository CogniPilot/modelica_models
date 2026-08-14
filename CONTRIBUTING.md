# Modelica library conventions

This library follows the conventions used by the Modelica Standard Library
(MSL) while keeping its public mathematical API stable.

## Package storage

- A public class may have its own `<ClassName>.mo` file. This is the normal MSL
  layout for independently useful and documented models, records, and
  functions (for example, the functions in `Modelica.Mechanics.MultiBody.Frames`).
- Closely related small declarations may instead be defined directly in their
  owning `package.mo`. Private helper functions should normally remain in the
  class that owns them rather than becoming public files.
- Do not split or combine files solely to enforce one layout. Choose the
  boundary that makes the public package easiest to navigate; moving a class
  between the two layouts must not change its fully qualified Modelica name.
- Every directory package that has child `.mo` files must have a complete
  `package.order`. Keep related declarations together and put prerequisites
  before the classes that primarily consume them.
- The `within` clause must match the filesystem package path exactly.

## Names and physical quantities

- Use `UpperCamelCase` for packages, models, blocks, connectors, records, and
  types. Use `lowerCamelCase` for functions, components, parameters, and local
  variables. Established Lie-group names such as `exp_map` remain compatible
  API; new aliases should use the project convention rather than multiplying
  spellings opportunistically.
- Public physical signals must declare `unit` attributes and include the frame
  and convention in the name or documentation. Navigation uses world ENU,
  body FLU, scalar-first Hamilton quaternions, and radians.
- Avoid unexplained decimal encodings of mathematical constants. Use symbolic
  expressions such as `armLength / sqrt(2.0)`.

## Interfaces and deployable tasks

- Reusable component boundaries use typed connectors and `connect` equations.
  Direct field equations are appropriate only when mapping between genuinely
  different message shapes.
- Transport schemas do not appear in algorithm packages. Pure-Modelica records
  in `Avionics` are the contract between drivers, estimators, and controllers.
- Every navigation estimator extends `Avionics.PartialNavigationEstimator`.
  Its covariance and internal state remain private; the public estimate and
  status records are algorithm-neutral.
- A deployable task must translate with OpenModelica and export as the actual
  named Rumoca `galec-production` eFMU target. Do not substitute a simpler
  surrogate in the export check.
- Test task behavior first in a deterministic ideal-RTOS Modelica composition,
  then run the same exported task boundary in the Rumoca/target-RTOS harness.

## Documentation and tests

- Give every public class a one-line description. Add `Documentation(info=...)`
  when frame conventions, equations, initialization, timing, or limitations
  cannot be understood from the declaration alone.
- Add assertions for mathematical invariants and regression tests for compiler
  boundaries. A Rumoca-only rejection of valid Modelica that OpenModelica
  accepts should be minimized and fixed in Rumoca, not hidden by scalarizing a
  vector or matrix algorithm.
