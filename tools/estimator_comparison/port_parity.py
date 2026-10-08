#!/usr/bin/env python3
"""Compare external port/native traces without rounding away floating-point bits."""

import argparse
import csv
from itertools import zip_longest
import json
import math
from pathlib import Path
import re
import struct

from manifest import digest


def records(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != ["case", "step", "field", "precision", "bits"]:
            raise ValueError("Expected case,step,field,precision,bits trace columns")
        seen = set()
        for row in reader:
            if None in row or not all(row.values()):
                raise ValueError(f"Missing or extra trace columns: {row}")
            key = tuple(row[name] for name in ("case", "step", "field"))
            precision = row["precision"]
            width, format_code = {"float32": (8, ">f"), "float64": (16, ">d")}.get(
                precision, (0, "")
            )
            if (
                not all(key)
                or key in seen
                or not width
                or not re.fullmatch(rf"[0-9a-fA-F]{{{width}}}", row["bits"])
            ):
                raise ValueError(f"Invalid or duplicate trace record: {row}")
            seen.add(key)
            bits = int(row["bits"], 16)
            value = struct.unpack(format_code, bytes.fromhex(row["bits"]))[0]
            if not math.isfinite(value):
                raise ValueError(f"Nonfinite trace value: {key}")
            yield key, precision, bits, value


def ordered_bits(bits, precision):
    sign = 1 << (31 if precision == "float32" else 63)
    return sign - (bits & (sign - 1)) if bits & sign else sign + bits


def compare(native, modelica):
    result = dict(
        components=0,
        bitwise_differences=0,
        signed_zero_differences=0,
        max_absolute_error=0.0,
        max_scaled_error=0.0,
        max_ulp_distance=0,
        first_difference=None,
        by_field_group={},
    )
    for reference, actual in zip_longest(records(native), records(modelica)):
        if reference is None or actual is None or reference[:2] != actual[:2]:
            raise ValueError("Trace keys, precision or component counts differ")
        key, precision, expected_bits, expected = reference
        _, _, actual_bits, value = actual
        group = key[2].split("[", 1)[0]
        counts = result["by_field_group"].setdefault(
            group, dict(components=0, bitwise_differences=0)
        )
        counts["components"] += 1
        result["components"] += 1
        if actual_bits == expected_bits:
            continue
        counts["bitwise_differences"] += 1
        result["bitwise_differences"] += 1
        result["signed_zero_differences"] += expected == value == 0
        difference = abs(value - expected)
        ulps = abs(
            ordered_bits(actual_bits, precision)
            - ordered_bits(expected_bits, precision)
        )
        result["max_absolute_error"] = max(result["max_absolute_error"], difference)
        result["max_scaled_error"] = max(
            result["max_scaled_error"], difference / (1 + abs(expected))
        )
        result["max_ulp_distance"] = max(result["max_ulp_distance"], ulps)
        if result["first_difference"] is None:
            result["first_difference"] = dict(
                zip(("case", "step", "field"), key, strict=True),
                precision=precision,
                native_bits=f"{expected_bits:0{8 if precision == 'float32' else 16}x}",
                modelica_bits=f"{actual_bits:0{8 if precision == 'float32' else 16}x}",
                native_value=expected,
                modelica_value=value,
                ulp_distance=ulps,
            )
    if not result["components"]:
        raise ValueError("Empty parity traces")
    result["bitwise_equal"] = result["bitwise_differences"] == 0
    result["native_trace_sha256"] = digest(native)
    result["modelica_trace_sha256"] = digest(modelica)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("native", "modelica", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    for name in ("native-input-sha256", "modelica-input-sha256"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--native-revision", required=True)
    parser.add_argument("--port-upstream-revision", required=True)
    parser.add_argument("--scope", action="append", required=True)
    args = parser.parse_args()
    if (
        not re.fullmatch(r"[0-9a-f]{64}", args.native_input_sha256)
        or args.native_input_sha256 != args.modelica_input_sha256
    ):
        parser.error("Parity requires identical, declared input hashes")
    if (
        not re.fullmatch(r"[0-9a-f]{40}", args.native_revision)
        or args.native_revision != args.port_upstream_revision
    ):
        parser.error("Parity requires the same immutable upstream revision")
    if args.output.exists():
        parser.error("Refusing to overwrite existing parity evidence")
    result = dict(
        upstream_revision=args.native_revision,
        input_sha256=args.native_input_sha256,
        supported_scope=args.scope,
        **compare(args.native, args.modelica),
        note="Exact equality applies only to the supplied components, cases and build. Kernel traces do not establish complete flight-estimator parity. Input and revision declarations require adapter provenance; this checker cannot verify how an external process produced a trace.",
    )
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps(result, allow_nan=False))
    return 0 if result["bitwise_equal"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
