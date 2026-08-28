# One record-valued call is materialized once per reader

`makePair` assigns one record field from a call and then reads that field
twice. The prior field's right-hand side is stored and cloned at every later
read, so a single `expensive(value)` call tree reaches the DAE three times.

Reproduced with `rumoca` `d4d80fbb`, the models CI pin:

    rumoca compile RecordCallCardinality.mo \
      --model RecordCallCardinality.Outer \
      --emit dae-json --output /tmp/record-call-cardinality.dae.json

The emitted DAE carries three `call` nodes naming `RecordCallCardinality.expensive`
where the algorithm calls it once:

    {"call": {"owner": 10, "function": 0, "output": 0, "operand_count": 1}}
    {"call": {"owner": 12, "function": 0, "output": 0, "operand_count": 1}}
    {"call": {"owner": 14, "function": 0, "output": 0, "operand_count": 1}}

## Why this is recorded here

It is the compile-time and run-time cost driver behind the fusion horizon's
re-base. `docs/delayed-fusion-horizon-wcet.md` measures a re-base at tens of
millions of executed instructions where the algebra needs about 33,000, and
attributes it to exactly this multiplication: a `Delta` carries 56 fields, so a
composition written inside a loop is evaluated about 35 times per iteration.

`tools/rumoca_acceptance.py` gates this file as the `record-call-cardinality`
row, and the WCET re-base delta as the `wcet-oracle` row.
