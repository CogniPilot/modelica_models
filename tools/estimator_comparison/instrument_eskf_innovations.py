"""Add read-only per-update observations to an owned generated ESKF C copy."""

import argparse
import json
from pathlib import Path
import re
import shutil

from manifest import digest


LEAVES = {
    "correctGps": ('"gps"', "6"),
    "correctGpsPosition": ('"gps_position"', "3"),
    "correctGpsVelocity": ('"gps_velocity"', "3"),
    "correctOpticalFlow": ('"optical_flow"', "2"),
    "correctBarometer": ('"barometer"', "1"),
    "correctMagnetometer": (
        'useEquivariantVector ? "magnetic_vector" : "magnetic_heading"',
        "useEquivariantVector ? 3 : 1",
    ),
}


def function_span(text, name):
    matches = list(
        re.finditer(
            r"(?m)^static void " + re.escape(name) + r"\(\n[^;{}]*\) \{\n", text
        )
    )
    if len(matches) != 1:
        raise ValueError(f"Require one actual generated definition: {name}")
    start = matches[0].end()
    tokens = re.compile(
        r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.S
    )
    depth = 1
    for token in tokens.finditer(text, start):
        if token[0] == "{":
            depth += 1
        elif token[0] == "}":
            depth -= 1
            if depth == 0:
                return start, token.start()
    raise ValueError("Unterminated generated function")


def instrument(text):
    for name, (sensor, dimension) in LEAVES.items():
        start, end = function_span(text, name)
        body = text[start:end]
        if "return;" in body:
            raise ValueError("Early return would bypass the update observation")
        text = (
            text[:start]
            + f"    beginEskfInnovation({sensor}, {dimension}, measurement_timestamp_s, measurementAge_s, innovationGate);\n"
            + body
            + "    finishEskfInnovation(ctx->accepted, ctx->rejectionReason, ctx->normalizedInnovationSquared);\n"
            + text[end:]
        )
    names = sorted(
        set(re.findall(r"static void (correctLinear_specialization_\d+)\(", text))
    )
    if len(names) != 4:
        raise ValueError("Require the four audited dense correction specializations")
    for name in names:
        start, end = function_span(text, name)
        body = text[start:end]
        signature = text[text.rfind("static void ", 0, start) : start]
        dimension = int(re.search(r"const float residual\[(\d+)\]", signature)[1])
        marker = re.compile(
            r"(?m)^( +)solveSPD_specialization_\d+\(\n +self,\n +\(\*([A-Za-z_]\w*)\),"
        )
        matches = list(marker.finditer(body))
        if len(matches) != 1 or dimension not in (1, 2, 3, 6):
            raise ValueError("Require the actual pre-solve innovation covariance")
        point = matches[0].start()
        indent = matches[0][1]
        covariance = matches[0][2]
        body = (
            "    if (eskf_observation.sensor && predicted_useSquareRootCovariance) abort();\n"
            + body[:point]
            + f"{indent}captureEskfInnovation({dimension}, residual, &measurementCovariance[0][0], &(*{covariance})[0][0]);\n"
            + body[point:]
        )
        text = text[:start] + body + text[end:]
    return '#include "eskf_innovation_dump.h"\n' + text


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output_source.exists() or args.output.exists():
        raise ValueError("Choose new owned source and evidence paths")
    args.output_source.parent.mkdir(parents=True, exist_ok=True)
    args.output_source.write_text(instrument(args.source.read_text()))
    header = Path(__file__).with_name("eskf_innovation_dump.h")
    shutil.copyfile(header, args.output_source.with_name(header.name))
    args.output.write_text(
        json.dumps(
            dict(
                original_sha256=digest(args.source),
                observed_sha256=digest(args.output_source),
                header_sha256=digest(header),
                functions=list(LEAVES),
                scope="Read-only generated dense ESKF update observations. Capture actual residual/S before the SPD solve and NIS/outcome after each leaf. Root mode rejected. State and full-covariance parity must be verified separately.",
            ),
            indent=2,
        )
        + "\n"
    )
