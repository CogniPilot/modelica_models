# A member of a package-level record constant cannot be referenced

Every member access on a package-level constant of **record** type is reported
as an unresolved Flat reference, however the value is supplied. A **scalar**
constant in the same package resolves.

Reproduced with `rumoca` `d4d80fbb`:

    rumoca compile PackageRecordConstant.mo \
      --model PackageRecordConstant.ReadsModifiedRecord

    [ED008] unresolved Flat reference
            `PackageRecordConstant.Basic.Modified.h_off`

| model | what it reads | result |
| --- | --- | --- |
| `ReadsScalarConstant` | `Basic.Scalar`, a `constant Real` | compiles, balanced |
| `ReadsModifiedRecord` | `Basic.Modified.h_off`, value from a modification on the declaration | `ED008` |
| `ReadsConstructedRecord` | `Basic.Constructed.h_off`, value from a record-constructor binding | `ED008` |

So it is not the modification form and it is not the constant itself. It is
member access on the record.

## Why this is recorded here

It is one of the two genuinely-new defects the MSL.Fluid frontier measurement
turned up (`tools/fluid-frontier/`), and it blocks three Media models outright:

| model | site |
| --- | --- |
| `Modelica.Media.Examples.IdealGasH2O` | `Media/IdealGases/Common/package.mo:221`, `data.name` where `data` is `constant DataRecord data` |
| `Modelica.Media.Examples.ReferenceAir.DryAir2` | `Media/Air/ReferenceAir.mo:1474`, `Air_Utilities.Basic.Constants.h_off` |
| `Modelica.Media.Examples.ReferenceAir.Inverse_sh_T` | the same `h_off` |

It is also the same root as the eight `EF024` "flat variable is missing
structured identity" rows in the frontier CSV, three of which lose `molarMass`
off `fluidConstants[1]` -- a package constant that is an *array* of records.

MSL uses this shape everywhere a medium family carries its per-substance data,
so the Fluid frontier cannot move far past it. Note that the failure is
independent of `final`: in MSL's `FundamentalConstants` the members that fail
(`h_off`, `s_off`) happen to be the only non-`final` ones, which makes `final`
look like the discriminator. `ReadsModifiedRecord` here has no `final` anywhere
and `R_s` fails identically, so it is not.
