Rumoca 0.10.2 ESKF runtime memory reproducer
==========================================

ConstructionProbe.mo extends the existing standalone ESKF test harness.
It contains no vehicle plant, controller, mission guidance or long flight
trace. On source commit 42779ec it passes structural analysis (212 equations,
212 unknowns, no coupled blocks) at roughly 0.5 GiB peak RSS. Solve-IR export
also succeeds, producing a 41 MiB JSON artifact at roughly 8.4 GiB peak RSS.
That artifact has 62 guarded assignments. Simulation of just 0.005 s aborts
on allocation failure under a 12 GiB address-space limit, through both the
released Python binding and released CLI. The actual manual-flight model
also aborts with the same limit and short requested time span.

The released runtime libraries are stripped; enabling RUST_BACKTRACE cannot
produce a useful backtrace at exhaustion because another allocation fails.
This reproducer bounds the investigation to the later simulation path; it
does not yet locate a specific allocation or prove whether it is in runtime
construction or initial event evaluation. It is not a passing mission or a
reason to change the original qualification envelope.

From the repository root, with its pinned Nix development environment:

  nix develop -c python3 tools/rumoca-repros/eskf-runtime-memory/probe.py \
    --root "$PWD" --mode minimal

The probe imposes a process-local 12 GiB RLIMIT_AS, disables core dumps,
prints stage timestamps/peak RSS, and requests only 0.005 s by default.
--mode mission instead compiles the original RDD2 scenario but keeps that
short diagnostic duration. Neither invocation is full qualification.

For structural analysis or a Solve-IR export, use the pinned rumoca CLI with
the repository as --source-root and ConstructionProbe.mo as the file:

  rumoca sim tools/rumoca-repros/eskf-runtime-memory/ConstructionProbe.mo \
    --model ConstructionProbe --source-root "$PWD" --inspect structure

  rumoca compile tools/rumoca-repros/eskf-runtime-memory/ConstructionProbe.mo \
    --model ConstructionProbe --source-root "$PWD" --emit solve-json \
    --output "$HOME/scratch/modelica_models/ci-construction-probe/solve.json"

Keep outputs, temporary files and owned caches under $HOME/scratch. The
durable logs and artifact fingerprint are in
docs/reviews/2026-10-08/ci-rdd2-memory/construction-*. A SIGABRT exit (134)
and explicit allocation-failure message override the timer's misleading
trailing Exit status 0. The source checkout and native estimator cores are
not altered by these probes.
