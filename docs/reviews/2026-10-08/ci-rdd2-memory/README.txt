RDD2 qualification remains unresolved
====================================

Main 7522ada passed Modelica Regression Tests and CUBS2 Mission Qualification
in GitHub run 37726702908. RDD2 terminated during the 28-second manual-flight
simulation before waypoint qualification. Retrying the failed job on the
same commit failed at the same stage, again reporting cancellation without
a Modelica diagnostic. This is not a green CI result.
https://github.com/CogniPilot/modelica_models/actions/runs/37726702908

Follow-up main 9163845 also passed Modelica Regression Tests and CUBS2, but
RDD2 again terminated during manual-flight simulation. GitHub run
37729385989 is terminal failure. main-9163845-rdd2.txt retains the decisive
log lines: simulation starts at 05:06:08 UTC, then the operation is canceled
at 05:07:20 UTC with a resource-tracker semaphore warning. There is no
hosted OOM diagnostic or BrokenProcessPool traceback in that log, so its
precise termination cause is not proven by the hosted evidence. Serialization
and launcher failure handling did not resolve this CI failure.
https://github.com/CogniPilot/modelica_models/actions/runs/37729385989

Run 37731292272 on subsequent main 42779ec is also terminal failure, with
Modelica Regression Tests and CUBS2 successful. RDD2 again reports cancellation
during manual simulation; main-42779ec-rdd2.txt retains its decisive lines.
https://github.com/CogniPilot/modelica_models/actions/runs/37731292272

An isolated local checkout at that commit reproduced excessive memory
growth with unmodified Rumoca 0.10.2. The local earlyoom service terminated
the owned worker at 52169 MiB RSS. The surrounding multiprocessing.Pool
replaced the dead worker but left the original task waiting indefinitely;
the parent was then explicitly interrupted. The time log is not a passing
simulation, and its trailing Exit status field does not override SIGINT.
earlyoom.txt records the corroborating journal observation. No machine
swap configuration or memory service was changed.

Qualification now uses ProcessPoolExecutor with the same spawn context and
one fresh process per mission. It reports an abrupt native worker exit as
BrokenProcessPool instead of waiting indefinitely. Launcher-only fixtures
verify both successful result transfer and abrupt os._exit(17) for both
manual and waypoint launchers. launchers.txt is not flight evidence: the
success callbacks are fixtures. All seven existing waypoint negative-control
tests, Ruff and whitespace checks passed after this change.

The hosted RDD2 worker count is reduced from four to one. All scenarios,
durations, sensor settings and acceptance checks remain. This avoids
multiplying peak demand when a single mission can run; it does not solve
the already observed single-mission memory problem. No large speculative
swap allocation, reduced acceptance envelope or skipped mission is added.

Diagnostic alternatives, not production changes
-----------------------------------------------
Simpler scalar Givens loops passed the 412-case generated-C QR regression
and Modelica root assertions but showed the same mission memory growth;
the mission was interrupted. Several equivalent loop arrangements were
rejected by Rumoca's checked function-conditional construction.

A Householder alternative passed 412 generated-C QR cases and root
assertions. Initial expression forms exposed GALEC projection limitations;
an explicit scale reduction and rank-one comprehension exported correctly.
The full mission still grew beyond 34 GiB and was interrupted. These
numerical results do not qualify the mission or establish a CI fix.

A diagnostic stub bypassing all QR calculations also grew to roughly
47 GiB before explicit interruption. It is not a valid square-root filter
and was never committed or used for an estimator comparison. This result
shows that changing the QR arithmetic alone is insufficient. The isolated
checkout has been restored to the real published implementation. All raw
prototype sources/builds remain under owned scratch storage.

OpenModelica was also tried on the actual main Modelica mission with the
proper OPENMODELICALIBRARY. It rejected an independent subsystem with 1984
equations and 1967 variables and did not build a simulation. Its unsuccessful
log is retained; it is not an alternative passing qualification result.

The next task is to isolate the Rumoca construction memory growth using
smaller composed estimator models and determine whether a faithful source
refactoring or upstream compiler fix can bound it. Native generated-C flight
replays and their covariance parity checks remain separate evidence; they
cannot substitute for a closed-loop RDD2 mission qualification.

A smaller reproducer now exists at tools/rumoca-repros/eskf-runtime-memory.
It extends only the existing ESKF harness. Structural analysis completes at
about 0.5 GiB RSS, and Solve-IR export completes at about 8.4 GiB RSS. Both
released CLI and Python simulation abort under an explicit process-local
12 GiB address-space cap even with a requested span of only 0.005 s. The
same short request on the actual manual mission also aborts. The paired
construction-*.txt logs and construction-summary.json retain stages, process
results and the 41 MiB Solve-IR artifact's hash. This isolates the symptom
from the full vehicle and long trace, but the exact later simulation
allocation or initial-event operation is not yet identified. These bounded
diagnostics do not qualify the original mission and do not change machine
swap, memory services, source models or acceptance checks.

The disabled startup-rest pressure option and its frozen-data study were
published separately as 3bf5bfb. Neither that study nor these launcher changes
claim ESKF superiority, matched native effective R/Q, complete NIS, a full
port, or successful main qualification. The broader comparison goal remains
active. Large artifacts belong in
$HOME/scratch/modelica_models/ci-qualification-main-results.
