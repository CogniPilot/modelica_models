import hashlib
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
REPO = Path.cwd()
sys.path.insert(0, str(ROOT / "tools"))
from compare_exposure import eskf
from check_joint_replay_covariance import check

sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
build = Path.home() / "scratch/modelica_models/joint-barometer/build-v2"
base = Path.home() / "scratch/modelica_models/ekf3-imu-integrity/fix-study"
stationary = Path.home() / "scratch/modelica_models/stationary-readiness"
reference = json.loads((base / "pilot.json").read_text())
stationary_reference = json.loads((stationary / "pilot.json").read_text())
original_build = json.loads((REPO / "docs/reviews/2026-10-08/common-sensor-noise/replay-build.json").read_text())
noise = json.loads((REPO / "docs/reviews/2026-10-07/native-releases/native-exposure.json").read_text())["imu_noise_density"]
declaration = json.loads((ROOT / "declaration.json").read_text())
assert sha(ROOT / "gauge.c") == declaration["candidate_generated_sha256"]
variants = {"horizon": "horizon-rest", "retrodiction": "retrodiction-rest", "horizon_joint": "horizon-rest-joint", "retrodiction_joint": "retrodiction-rest-joint"}
compiler = "/nix/store/29qjlshvklnyr67nhpprzgb9igmsfsjj-gcc-wrapper-15.2.0/bin/gcc"
compile_command = [compiler, "-O2", "-I" + str(build / "Vehicles_Rdd2_NavigationEstimator/Vehicles_Rdd2_NavigationEstimator/ProductionCode"), "-c", str(ROOT / "gauge.c"), "-o", str(ROOT / "gauge.o")]
subprocess.run(compile_command, check=True)
binaries = {}
for rest in (False, True):
    for name, variant in variants.items():
        config = original_build["variants"][variant]
        for obj, expected in config["generated_object_sha256"].items():
            assert sha(build / obj) == expected
        dependencies = ["Vehicles_Rdd2_NavigationEstimator", "Tests_PreintegrationReplay"]
        if name.startswith("horizon"):
            dependencies += ["Estimation_FusionHorizon_OutputPredictor", "Estimation_FusionHorizon_AidingBuffer"]
        key = ("stationary_" if rest else "") + name
        binary = ROOT / key
        command = [compiler, "-pipe", "-O2", *config["definitions"], *(["-DSTATIONARY_IMU_MODEL"] if rest else []), *["-I" + str(build / dep / dep / "ProductionCode") for dep in dependencies], str(ROOT / "tools/replay.c"), str(ROOT / "gauge.o"), *[str(build / (dep + ".o")) for dep in dependencies[1:]], str(build / "rumoca_galec_kernels.o"), "-lm", "-o", str(binary)]
        subprocess.run(command, check=True)
        binaries[key] = dict(command=command, binary_sha256=sha(binary))
(ROOT / "build.json").write_text(json.dumps(dict(compile_command=compile_command, gauge_object_sha256=sha(ROOT / "gauge.o"), binaries=binaries), indent=2) + "\n")
assert not (ROOT / "pilot.json").exists()
scores = []
for scenario in ("gps", "denied", "transition"):
    capture = base / "pilot" / scenario / "capture"
    assert {name: sha(capture / name) for name in reference["input_sha256"]} == reference["input_sha256"]
    transport = next(s["transport"] for s in reference["scores"] if s["scenario"] == scenario)
    for rest in (False, True):
        control = stationary_reference if rest else reference
        for name in variants:
            key = ("stationary_" if rest else "") + name
            work = ROOT / "pilot" / scenario / key
            work.mkdir(parents=True)
            options = SimpleNamespace(work=work, arm_after_s=120, delay_profile="exposure", seed=911, imu_noise_density=noise, mission_offset_s=107, measurement_noise=(1e-6, .075), accuracy_end_s=166.7, consistency_windows=reference["mission"]["windows"], retain_covariance=name.endswith("_joint"))
            assert sha(ROOT / key) == binaries[key]["binary_sha256"]
            score = eskf(ROOT / key, capture, scenario, options, True, transport)
            if name.endswith("_joint"):
                score["full16_covariance"] = check(work / "covariance.csv", name.startswith("horizon"), 117, 166.7)
            previous = next(s for s in control["scores"] if s["name"] == name and s["scenario"] == scenario)
            changes = {metric: 100 * (value / previous["flight"][metric] - 1) for metric, value in score["flight"].items() if metric.endswith(("rmse_m", "rmse_m_s", "rmse_deg")) and previous["flight"][metric] > 0}
            scores.append(dict(name=name, stationary=rest, scenario=scenario, flight_percent_change=changes, **score))
            (ROOT / "pilot.json").write_text(json.dumps(dict(complete=False, scores=scores), indent=2) + "\n")
            print(scenario, key, changes, flush=True)
assert len(scores) == 24
(ROOT / "pilot.json").write_text(json.dumps(dict(complete=True, scores=scores), indent=2) + "\n")
