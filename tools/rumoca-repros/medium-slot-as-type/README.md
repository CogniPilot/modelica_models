# A member type of a replaceable package collapses to the package slot

A component declared `Medium.Pressure`, where `Medium` is a replaceable
package, is reported as having the *package slot* as its type. At extent zero
the same loss presents as the member not existing at all.

Reproduced with `rumoca` `d4d80fbb`:

    rumoca compile MediumSlotAsType.mo --model MediumSlotAsType.ScalarPort

    [ET002] type mismatch: expected `Real`, found
            `MediumSlotAsType.FluidPort.Medium`

    rumoca compile MediumSlotAsType.mo --model MediumSlotAsType.ZeroExtentPorts

    [ET001] unknown member `m_flow` on component reference `ports.m_flow`
            of type `MediumSlotAsType.VesselPorts`

| model | shape | result |
| --- | --- | --- |
| `ScalarPort` | one connector with a replaceable package | `ET002`, the slot stands where `Real` should |
| `ZeroExtentPorts` | `[0]` array of that connector | `ET001`, the member is gone |
| `ZeroExtentPlainPorts` | `[0]` array of a connector with no replaceable package | compiles, balanced |

The third row is the control. Zero extent on its own is handled; zero extent
plus a package slot is not, and the member is not reported missing because the
array is empty but because its element type never got its members.

## Why this is recorded here

It is one of the two genuinely-new defects the MSL.Fluid frontier measurement
turned up (`tools/fluid-frontier/`), and between its two faces it blocks
sixteen models on that corpus.

The `ET002` face blocks ten models. Eight are `Modelica.Fluid.Sensors`
models -- `Pressure`, `Density`, `Temperature`, `SpecificEnthalpy`,
`SpecificEntropy`, `MassFractions`, `RelativePressure`, `RelativeTemperature`
-- each reporting `expected Modelica.Fluid.Interfaces.FluidPort.Medium, found
Integer`. The other two are
`Modelica.Fluid.Vessels.BaseClasses.HeatTransfer.IdealHeatTransfer` and
`Modelica.Fluid.Pipes.BaseClasses.HeatTransfer.IdealFlowHeatTransfer`, which
report `...PartialHeatTransfer.Medium` where `Medium.Temperature` was written.

The `ET001` face blocks six models that declare a connector array whose extent
defaults to zero: five `AST_BatchPlant` `Test` models, whose tanks carry
`VesselFluidPorts_a topPorts[nTopPorts]` with `nTopPorts = 0`, and
`Modelica.Fluid.Fittings.MultiPort`, which carries
`FluidPorts_b ports_b[nPorts_b]` with `nPorts_b = 0`.

That second face is the zero-extent dependency the Fluid P0 boundary review
named as a missed one: the checked-empty aggregate path has to carry
zero-extent members through connectors and signatures rather than reading
absence of scalar leaves as absence of the member. This file is the smallest
statement of it, and it shows the dependency is not separable from effective
specialization -- the empty array only loses the member when a package slot is
involved.
