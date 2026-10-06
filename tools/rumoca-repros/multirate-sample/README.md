# Reading a slower sampled producer

Rumoca 0.10.2 rejects this ordinary Boolean `sample(start, interval)` model
while constructing the canonical DAE:

```text
[ED020] canonical DAE construction rejected an invalid operation:
variable identity 0 is not owned by clock identity 0
```

The error points at `held := source`. Changing the consumer interval from
0.00125 to 0.01 makes the same model compile. These are discrete `when`
clauses, not synchronous `Clock` partitions: the consumer should read the
producer's current held value between its releases.

Reproduce with the repository's pinned compiler:

```sh
rumoca compile tools/rumoca-repros/multirate-sample/MultiRate.mo \
  --model MultiRate --emit dae-json --output "$TMPDIR/MultiRate.json"
```

`Estimation.FusionHorizon.HorizonEstimator` crosses this same boundary when
its 800 Hz sampled clause reads the 100 Hz filter's published position at
`filterPositionHeld_m := filter.estimate.positionWorldEnu_m`. The pin reports
variable identity 573 and clock identity 0 at that assignment. The previous
0.10.0 pin stopped earlier with a Boolean connector balance excess of 26;
that earlier diagnostic is recorded in `../connector-boolean-balance/`.

CI checks this exact new diagnostic and source location. A different failure
is an error, and successful lowering is reported so this model can become a
required export. OpenModelica's horizon assertion suite remains required.
