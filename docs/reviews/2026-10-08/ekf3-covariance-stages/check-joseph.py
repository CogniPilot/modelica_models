import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

repository = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(repository / "tools/estimator_comparison"))
from diagnose_ekf3_covariance_stages import matrix, jacobian, scalar_covariances

parser = argparse.ArgumentParser(
    description="Check the recorded native gains with the generated production Joseph function."
)
parser.add_argument(
    "--artifact-root",
    type=Path,
    default=Path.home() / "scratch/modelica_models/ekf3-covariance-stages",
)
parser.add_argument(
    "--generated-code",
    type=Path,
    default=Path.home()
    / "scratch/modelica_models/joint-barometer/probe-v2/Tests_BarometerConsiderReplay/ProductionCode",
)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
if args.output.exists():
    raise ValueError("Choose a fresh evidence path")
root = args.artifact_root
function = ctypes.CDLL(str(root / "joseph_probe.so")).native_gain_joseph
pointer = ctypes.POINTER(ctypes.c_float)
function.argtypes = [pointer] * 5
function.restype = ctypes.c_int
cases = []
for scenario in ("gps", "transition"):
    rows = np.genfromtxt(
        root / (scenario + "-verified/stages.csv"),
        delimiter=",",
        names=True,
        dtype=None,
        encoding="utf-8",
    )
    evidence = json.loads((root / (scenario + "-verified.json")).read_text())
    failures = evidence["analysis"]["scalar_positive_to_invalid_updates"]
    for failure in failures:
        (row,) = rows[
            (rows["stage"] == "scalar.before")
            & (rows["publication_us"] == round(failure["publication_s"] * 1e6))
            & (rows["axis"] == failure["axis"])
        ]
        prior = matrix(row)
        transform = jacobian(row)
        native_gain = np.array([row[f"k{i}"] for i in range(24)])
        observed_state = int(row["axis"]) + 4
        observation = np.eye(24)[observed_state] @ np.linalg.pinv(transform)
        np.testing.assert_allclose(
            observation @ transform, np.eye(24)[observed_state], atol=1e-13
        )
        common_prior = transform @ prior @ transform.T
        common_gain = transform @ native_gain
        factor = np.eye(15) - np.outer(common_gain, observation)
        noise = float(row["noise_variance"])
        arguments = [
            np.ascontiguousarray(v, dtype=np.float32)
            for v in (factor, common_prior, common_gain[:, None], [[noise]])
        ]
        output = np.empty((15, 15), dtype=np.float32)
        assert function(*[v.ctypes.data_as(pointer) for v in (*arguments, output)]) == 0
        reference = arguments[0].astype(float) @ arguments[1].astype(float) @ arguments[
            0
        ].astype(float).T + float(arguments[3][0, 0]) * np.outer(
            arguments[2][:, 0].astype(float), arguments[2][:, 0].astype(float)
        )
        symmetric = (np.float32(0.5) * (output + output.T)).astype(float)
        np.linalg.cholesky(symmetric)
        error = float(np.max(np.abs(output - reference)))
        assert error < 1e-7
        _, native_joseph = scalar_covariances(prior, native_gain, observed_state, noise)
        unrounded = transform @ native_joseph @ transform.T
        cases.append(
            dict(
                scenario=scenario,
                publication_s=failure["publication_s"],
                fusion_s=failure["fusion_s"],
                common_input_min_eigenvalue=float(
                    np.linalg.eigvalsh(arguments[1].astype(float)).min()
                ),
                generated_output_max_asymmetry=float(np.max(np.abs(output - output.T))),
                generated_symmetric_output_cholesky_valid=True,
                symmetry_arithmetic="float32: 0.5 * (A + A.T)",
                generated_symmetric_output_min_eigenvalue=float(
                    np.linalg.eigvalsh(symmetric).min()
                ),
                maximum_absolute_error_vs_float_input_double_oracle=error,
                maximum_absolute_error_vs_unrounded_native_joseph=float(
                    np.max(np.abs(output - unrounded))
                ),
            )
        )
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
code = args.generated_code
result = dict(
    cases=cases,
    rumoca="0.10.2",
    compiler="GCC 15.2.0 -O2",
    wrapper_sha256=sha(root / "joseph_probe.c"),
    library_sha256=sha(root / "joseph_probe.so"),
    generated_source_sha256={
        p.name: sha(p)
        for p in code.iterdir()
        if p.is_file() and p.suffix in (".c", ".h")
    },
    production_modelica_sha256={
        str(p): sha(repository / p)
        for p in (
            Path("LinearAlgebra/josephUpdate.mo"),
            Path("LinearAlgebra/transformCovariance.mo"),
        )
    },
    scope="Existing generated production Joseph function applied to the recorded native constrained-gain witness in the common 15D tangent. Float output is explicitly symmetrized as in correctLinear, never eigenvalue-clipped or regularized. This checks covariance algebra only, not a native firmware repair, a full ESKF replay of that gain policy, Lie-reset behavior, or overall estimator superiority.",
)
args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
print(json.dumps(result, indent=2))
