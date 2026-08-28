# Boolean connector components are not counted as sub-block unknowns

Rumoca reports a model unbalanced by exactly the number of BOOLEAN components
on the input connectors of its sub-blocks. The whole-record pass-through
equality contributes an equation per component, Boolean ones included, but the
Boolean components of the sub-block's input connector are not counted among the
unknowns those equations solve for.

Reproduced with `rumoca` `853791b5`:

    rumoca compile ConnectorBooleanBalance.mo \
      --model ConnectorBooleanBalance.Outer \
      --emit dae-json --output /tmp/connector-boolean-balance.dae.json

    [ED001] unbalanced model: 8 equations, 6 unknowns (balance = 2)

`Sample` carries two Booleans, a timestamp and a three-vector. Delete the two
Booleans and `Outer` balances.

## Why this is recorded here

It is the whole reason `Tests.HorizonEstimatorWiring` does not lower, and the
attribution matters because the model's own documentation used to blame the
wrong thing.

`Avionics.PartialNavigationEstimator` declares six input connectors -- IMU,
mocap, GPS, magnetometer, barometer, optical flow -- carrying fourteen Boolean
components between them: `valid` and `fresh` on each, plus `positionValid` and
`velocityValid` on GPS. Any block that instantiates a navigation estimator as a
sub-component and passes those connectors through is therefore reported
unbalanced by fourteen, and `Tests.HorizonEstimatorWiring` holds two harnesses,
which is the twenty-eight it reports.

Established by bisection on this tree at `853791b5`:

| model | balance |
| --- | --- |
| `Estimation.StrapdownINS.ESKF.Estimator` alone, as the top-level model | lowers |
| `Estimation.StrapdownINS.PartialEstimator` alone, as the top-level model | lowers |
| `Estimation.FusionHorizon.OutputPredictor` alone | lowers, all the way to `galec-production` |
| a twenty-line block whose only content is an ESKF sub-component and its pass-through equations | +14 |
| the same with the UKF instead | +14 |
| the same with all forty-seven estimator outputs consumed | +14 |
| `Estimation.FusionHorizon.HorizonEstimator` | +14 |
| `Tests.HorizonEstimatorWiring` | +28 |

Identical for both filters and unchanged by consuming the outputs, so it is a
property of the shared boundary and not of either filter, and nothing in
`Estimation.FusionHorizon` is implicated. The horizon block on its own is what
`tools/ci.py` lowers, and that is the largest subset that lowers today.
