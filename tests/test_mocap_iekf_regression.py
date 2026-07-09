from __future__ import annotations

import textwrap
from pathlib import Path

import rumoca as rm


ROOT = Path(__file__).resolve().parents[1]
MODEL_FILE = ROOT / "Estimation" / "Examples" / "MocapExternalOdometryIEKF.mo"
MODEL_NAME = "Estimation.Examples.MocapExternalOdometryIEKF"


def test_mocap_iekf_example_is_discrete_packet_wrapper() -> None:
    source = MODEL_FILE.read_text()

    assert "der(" not in source
    assert "input Real x_prev[157]" in source
    assert "output Real x_next[157]" in source
    assert "Estimation.MocapExternalOdometryIEKF.step" in source

    rm.Session(roots=[str(ROOT)]).load(MODEL_FILE, model=MODEL_NAME)


def test_mocap_iekf_discrete_step_lowers_for_rust_codegen(tmp_path: Path) -> None:
    model_path = tmp_path / "MocapIEKFStepScalarHarness.mo"
    model_path.write_text(_modelica_step_scalar_harness())

    model = rm.Session(roots=[str(ROOT), str(tmp_path)]).load(
        model_path, model="MocapIEKFStepScalarHarness"
    )
    codegen = model.codegen("rust-solve")
    generated = codegen.files[0].content

    assert "pub const Y_LEN: usize = 166;" in generated
    assert "pub const DERIVATIVE_LEN: usize = 166;" in generated


def _modelica_step_scalar_harness() -> str:
    return textwrap.dedent(
        """\
        within;
        function mocapIekfStepState
          input Real x_prev[157];
          input Real dt_s;
          input Real measurement_valid;
          input Real measurement_position_enu_m[3];
          input Real measurement_attitude_wxyz[4];
          output Real x_next[157];
        protected
          Real correction_accepted;
        algorithm
          (x_next, correction_accepted) := Estimation.MocapExternalOdometryIEKF.step(
            x_prev,
            dt_s,
            measurement_valid,
            measurement_position_enu_m,
            measurement_attitude_wxyz,
            {1.0e-5, 2.0e-2, 1.0e-6, 5.0e-3},
            0.010^2,
            0.004^2);
        end mocapIekfStepState;

        // Test-only adapter: rust-solve currently exposes derivative_rhs, so
        // one scalar from the discrete packet step is mapped through der(...).
        // The Estimation package itself stays der-free.
        model MocapIEKFStepScalarHarness
          Real slot[166](each start = 0.0);
        equation
          der(slot[1]) = mocapIekfStepState(
            slot[1:157],
            slot[158],
            slot[159],
            {slot[160], slot[161], slot[162]},
            {slot[163], slot[164], slot[165], slot[166]})[1];
          for i in 2:166 loop
            der(slot[i]) = 0.0;
          end for;
        end MocapIEKFStepScalarHarness;
        """
    )
