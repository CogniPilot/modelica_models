import argparse
import json
import resource
import time
from pathlib import Path
import rumoca as rum

parser = argparse.ArgumentParser()
parser.add_argument("--root", type=Path, required=True)
parser.add_argument("--mode", choices=("minimal", "mission"), required=True)
parser.add_argument("--seconds", type=float, default=0.005)
args = parser.parse_args()
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
resource.setrlimit(resource.RLIMIT_AS, (12 * 1024**3, 12 * 1024**3))
start = time.monotonic()


def stage(name):
    print(
        json.dumps(
            dict(
                stage=name,
                elapsed_s=time.monotonic() - start,
                max_rss_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            )
        ),
        flush=True,
    )


stage("before_compile")
if args.mode == "mission":
    session, model, config = rum.Session.from_scenario(
        str(args.root / "Vehicles/Rdd2/Test/rumoca-scenario.manual-flight.toml")
    )
else:
    session = rum.Session(roots=[str(args.root)])
    model = session.loads(
        "model ConstructionProbe extends Tests.StrapdownEstimatorInterfaceTests.Harness; end ConstructionProbe;",
        model="ConstructionProbe",
    )
    config = rum.SimConfig(solver="rk-like", dt=0.005, max_wall_seconds=60)
stage("after_compile")
print(model.summary(), flush=True)
stage("before_simulation")
model.simulate(t=(0, args.seconds), config=config)
stage("after_simulation")
