from __future__ import annotations

import subprocess
import textwrap
from pathlib import Path

import rumoca as rm

from tests.rust_codegen import normalize_rumoca_rust


ROOT = Path(__file__).resolve().parents[1]
ERROR_LEN = 61


def test_liegroups_core_identities_compile_and_evaluate(tmp_path: Path) -> None:
    model_path = tmp_path / "LieGroupsRegression.mo"
    model_path.write_text(_modelica_regression_model())

    model = rm.Session(roots=[str(ROOT), str(tmp_path)]).load(
        model_path, model="LieGroupsRegression"
    )
    codegen = model.codegen("rust-solve")

    generated_path = tmp_path / "liegroups_generated.rs"
    generated_path.write_text(normalize_rumoca_rust(codegen.files[0].content))

    harness_path = tmp_path / "liegroups_regression.rs"
    harness_path.write_text(_rust_harness())
    exe_path = tmp_path / "liegroups_regression"

    subprocess.run(
        ["rustc", "-Awarnings", "-O", str(harness_path), "-o", str(exe_path)],
        cwd=tmp_path,
        check=True,
    )
    subprocess.run([str(exe_path)], check=True)


def _modelica_regression_model() -> str:
    return textwrap.dedent(
        f"""\
        within;
        model LieGroupsRegression
          Real errors[{ERROR_LEN}](each start = 0.0);
        equation
          der(errors[1]) = LieGroups.SO2.product(0.7, LieGroups.SO2.inverse(0.7));
          der(errors[2]) = LieGroups.SO2.rotate(1.5707963267948966, {{1.0, 0.0}})[1];
          der(errors[3]) = LieGroups.SO2.rotate(1.5707963267948966, {{1.0, 0.0}})[2] - 1.0;

          der(errors[4]) = LieGroups.SE2.log_map(LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}}))[1] - 0.4;
          der(errors[5]) = LieGroups.SE2.log_map(LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}}))[2] + 0.2;
          der(errors[6]) = LieGroups.SE2.log_map(LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}}))[3] - 0.3;
          der(errors[7]) = LieGroups.SE2.product(
            LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}}),
            LieGroups.SE2.inverse(LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}})))[1];
          der(errors[8]) = LieGroups.SE2.product(
            LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}}),
            LieGroups.SE2.inverse(LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}})))[2];
          der(errors[9]) = LieGroups.SE2.product(
            LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}}),
            LieGroups.SE2.inverse(LieGroups.SE2.exp_map({{0.4, -0.2, 0.3}})))[3];

          der(errors[10]) = LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}))[1] - 0.12;
          der(errors[11]) = LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}))[2] + 0.08;
          der(errors[12]) = LieGroups.SO3.Quat.log_map(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}))[3] - 0.05;
          der(errors[13]) = LieGroups.SO3.Quat.product(
            LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}),
            LieGroups.SO3.Quat.inverse(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}})))[1] - 1.0;
          der(errors[14]) = LieGroups.SO3.Quat.product(
            LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}),
            LieGroups.SO3.Quat.inverse(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}})))[2];
          der(errors[15]) = LieGroups.SO3.Quat.product(
            LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}),
            LieGroups.SO3.Quat.inverse(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}})))[3];
          der(errors[16]) = LieGroups.SO3.Quat.product(
            LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}}),
            LieGroups.SO3.Quat.inverse(LieGroups.SO3.Quat.exp_map({{0.12, -0.08, 0.05}})))[4];

          der(errors[17]) = LieGroups.SO3.Mrp.log_map(LieGroups.SO3.Mrp.exp_map({{0.10, 0.07, -0.04}}))[1] - 0.10;
          der(errors[18]) = LieGroups.SO3.Mrp.log_map(LieGroups.SO3.Mrp.exp_map({{0.10, 0.07, -0.04}}))[2] - 0.07;
          der(errors[19]) = LieGroups.SO3.Mrp.log_map(LieGroups.SO3.Mrp.exp_map({{0.10, 0.07, -0.04}}))[3] + 0.04;

          der(errors[20]) = LieGroups.SE3.Quat.log_map(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}))[1] - 0.4;
          der(errors[21]) = LieGroups.SE3.Quat.log_map(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}))[2] + 0.2;
          der(errors[22]) = LieGroups.SE3.Quat.log_map(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}))[3] - 0.3;
          der(errors[23]) = LieGroups.SE3.Quat.log_map(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}))[4] - 0.08;
          der(errors[24]) = LieGroups.SE3.Quat.log_map(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}))[5] + 0.04;
          der(errors[25]) = LieGroups.SE3.Quat.log_map(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}))[6] - 0.06;
          der(errors[26]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[1];
          der(errors[27]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[2];
          der(errors[28]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[3];
          der(errors[29]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[4] - 1.0;
          der(errors[30]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[5];
          der(errors[31]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[6];
          der(errors[32]) = LieGroups.SE3.Quat.product(
            LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}}),
            LieGroups.SE3.Quat.inverse(LieGroups.SE3.Quat.exp_map({{0.4, -0.2, 0.3, 0.08, -0.04, 0.06}})))[7];

          der(errors[33]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[1] - 0.3;
          der(errors[34]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[2] + 0.1;
          der(errors[35]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[3] - 0.2;
          der(errors[36]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[4] + 0.05;
          der(errors[37]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[5] - 0.08;
          der(errors[38]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[6] - 0.12;
          der(errors[39]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[7] - 0.06;
          der(errors[40]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[8] + 0.03;
          der(errors[41]) = LieGroups.SE23.Quat.log_map(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}))[9] - 0.04;

          der(errors[42]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[1];
          der(errors[43]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[2];
          der(errors[44]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[3];
          der(errors[45]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[4];
          der(errors[46]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[5];
          der(errors[47]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[6];
          der(errors[48]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[7] - 1.0;
          der(errors[49]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[8];
          der(errors[50]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[9];
          der(errors[51]) = LieGroups.SE23.Quat.product(
            LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}}),
            LieGroups.SE23.Quat.inverse(LieGroups.SE23.Quat.exp_map({{0.3, -0.1, 0.2, -0.05, 0.08, 0.12, 0.06, -0.03, 0.04}})))[10];

          der(errors[52]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[1] - 1.05;
          der(errors[53]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[2] - 1.975;
          der(errors[54]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[3] + 0.425;
          der(errors[55]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[4] - 0.2;
          der(errors[56]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[5] + 0.1;
          der(errors[57]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[6] - 0.3;
          der(errors[58]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[7] - 1.0;
          der(errors[59]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[8];
          der(errors[60]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[9];
          der(errors[61]) = LieGroups.SE23.Quat.exp_mixed({{
            1.0, 2.0, -0.5, 0.2, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
            [0.0, 0.25; 0.0, 0.0])[10];
        end LieGroupsRegression;
        """
    )


def _rust_harness() -> str:
    return textwrap.dedent(
        f"""\
        #[path = "liegroups_generated.rs"]
        mod generated;

        const ERROR_LEN: usize = {ERROR_LEN};

        fn main() {{
            assert_eq!(generated::DERIVATIVE_LEN, ERROR_LEN);
            let y = vec![0.0; generated::Y_LEN];
            let p = vec![0.0; generated::P_LEN];
            let mut out = vec![0.0; generated::DERIVATIVE_LEN];
            generated::derivative_rhs(0.0, &y, &p, &mut out);

            let mut max_abs = 0.0_f64;
            let mut max_index = 0_usize;
            for (index, value) in out.iter().copied().enumerate() {{
                assert!(value.is_finite(), "error {{}} is not finite: {{}}", index + 1, value);
                let abs = value.abs();
                if abs > max_abs {{
                    max_abs = abs;
                    max_index = index + 1;
                }}
            }}
            assert!(
                max_abs < 1.0e-7,
                "max LieGroups residual at error {{}} was {{:.12e}}",
                max_index,
                max_abs
            );
        }}
        """
    )
