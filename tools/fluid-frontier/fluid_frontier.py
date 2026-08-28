#!/usr/bin/env python3
"""Measure how far Rumoca gets on the MSL Fluid frontier, and why it stops.

This is the ratchet scoreboard for the Fluid P0 campaign. It compiles a pinned
set of Modelica Standard Library 4.1.0 models -- every model under
`Modelica.Fluid.Examples`, the public component models of the seven Fluid
component packages, and the whole `Modelica.Media.Examples` ladder -- with one
`rumoca compile` invocation each, and writes one row per model saying what
happened and which architecture gap the failure evidences.

The point is a number that can only be moved by making the compiler better:

    N of 115 Fluid frontier models compile today.

This is NOT run by CI. It starts up to 115 compiler processes against a full
standard library and costs minutes. Run it by hand when a compiler branch
claims to have moved the frontier, and diff its CSV against the committed one.

    python3 tools/fluid-frontier/fluid_frontier.py \
        --rumoca /path/to/rumoca \
        --msl-root /path/to/ModelicaStandardLibrary-4.1.0 \
        --out tools/fluid-frontier/frontier.csv

`--msl-root` wants the RELEASE layout, the directory that holds
`Modelica 4.1.0/package.mo`, `ModelicaServices 4.1.0/` and `Complex.mo` -- the
same root `tools/rumoca_acceptance.py --msl-root` wants, and the same one
`cargo xtask verify corpus-pin` resolves. A bare `Modelica/` checkout is not
that layout and will fail every row identically, which is a rig error, not a
measurement.

The target set is pinned in `targets.tsv` next to this script so the scoreboard
compares like with like across runs. `--enumerate` regenerates it from the MSL
root; regenerating it against a different MSL version changes what the headline
number counts, so it is a deliberate, reviewable act.

Every failure is classified against the Fluid P0 gap taxonomy. The rules are
spelled out in `classify()` and each one names the evidence it matches on.
UNCLASSIFIED is a real bucket: a failure nobody has looked at is more useful
recorded as unexamined than filed under a guess.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import os
import re
import subprocess
import sys
import time
from pathlib import Path

# The compiler revision this scoreboard was measured with. A run against a
# different binary is still valid; the provenance simply has to be restated.
PIN = "d4d80fbb"

# The release-layout entry point every row is compiled through, exactly as
# `infra/verification/corpus-pin.json` names it for its own MSL rows.
ENTRY_POINT = "Modelica 4.1.0/package.mo"

ANSI = re.compile(r"\x1b\[[0-9;]*m")

# A diagnostic code is stable by its bare mnemonic suffix (SPEC_0008), so match
# the suffix and never the `rumoca::<phase>::` rendering around it.
CODE = re.compile(r"\[([EW][A-Z]{1,2}\d{3})\]")

# Which phase owns which mnemonic prefix (SPEC_0008 "Error Code Ranges").
PHASE_OF_PREFIX = {
    "EP": "parse",
    "ER": "resolve",
    "ET": "typecheck",
    "WT": "typecheck",
    "EI": "instantiate",
    "WI": "instantiate",
    "EF": "flatten",
    "ED": "dae",
    "EC": "codegen",
    "EM": "merge",
    "ES": "structural",
    "EL": "solve-lowering",
    "EX": "sim-runtime",
    "EG": "galec",
    "EGT": "galec-target",
    "EFM": "efmi",
}

BALANCE = re.compile(
    r"unbalanced model: (\d+) equations, (\d+) unknowns \(balance = (-?\d+)\)"
)

TIMEOUT_SECONDS = 120


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description="Measure the Rumoca MSL.Fluid frontier",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--rumoca", required=True, help="compiler binary to measure")
    parser.add_argument(
        "--msl-root",
        required=True,
        help="MSL release root: the directory holding `Modelica 4.1.0/package.mo`",
    )
    parser.add_argument(
        "--out", help="CSV to write (default: frontier.csv next to this script)"
    )
    parser.add_argument(
        "--targets",
        help="target list (default: targets.tsv next to this script)",
    )
    parser.add_argument(
        "--enumerate",
        action="store_true",
        help="regenerate the target list from --msl-root instead of measuring",
    )
    parser.add_argument(
        "--cache-dir", help="compiler cache root (default: a directory beside --out)"
    )
    parser.add_argument(
        "--logs", help="directory to keep one full compiler log per model"
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=min(8, os.cpu_count() or 4),
        help="concurrent compiler processes (default: min(8, cpus))",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=TIMEOUT_SECONDS,
        help=f"per-model ceiling in seconds (default: {TIMEOUT_SECONDS})",
    )
    parser.add_argument("--only", help="comma-separated model names to measure")
    parser.add_argument(
        "--reclassify",
        action="store_true",
        help="re-derive buckets from the logs kept by an earlier --logs run, "
        "without starting a compiler. Use it when a taxonomy rule changes: the "
        "measurement is the compiler's, the classification is ours, and only "
        "the second one should move.",
    )
    return parser.parse_args(argv)


# --------------------------------------------------------------------------
# Target enumeration
# --------------------------------------------------------------------------

OPEN_RE = re.compile(
    r"^\s*(?P<flags>(?:(?:final|inner|outer|replaceable|partial|redeclare|"
    r"encapsulated|impure|pure|expandable|operator|input|output)\s+)*)"
    r"(?P<kind>package|model|class|record|function|block|connector|type|operator)\s+"
    r"(?P<name>[A-Za-z_]\w*|'[^']*')"
)
END_RE = re.compile(r"^\s*end\s+([A-Za-z_]\w*|'[^']*')\s*;")

FLUID_COMPONENT_PACKAGES = (
    "Vessels",
    "Machines",
    "Pipes",
    "Valves",
    "Sources",
    "Sensors",
    "Fittings",
)


def strip_comments(text: str) -> str:
    """Blank out strings and comments, preserving every byte offset and line."""
    out, index, size = [], 0, len(text)
    while index < size:
        character = text[index]
        if character == '"':
            end = index + 1
            while end < size:
                if text[end] == "\\":
                    end += 2
                    continue
                if text[end] == '"':
                    break
                end += 1
            out.append(re.sub(r"[^\n]", " ", text[index : end + 1]))
            index = end + 1
        elif text.startswith("//", index):
            end = text.find("\n", index)
            end = size if end < 0 else end
            out.append(" " * (end - index))
            index = end
        elif text.startswith("/*", index):
            end = text.find("*/", index + 2)
            end = size if end < 0 else end + 2
            out.append(re.sub(r"[^\n]", " ", text[index:end]))
            index = end
        else:
            out.append(character)
            index += 1
    return "".join(out)


def scan(path: Path, prefix):
    """Yield (qualified name, kind, flags) for the classes one file declares."""
    text = strip_comments(path.read_text(encoding="utf-8", errors="replace"))
    stack, results = list(prefix), []
    for line in text.splitlines():
        ended = END_RE.match(line)
        if ended:
            if stack and stack[-1] == ended.group(1):
                stack.pop()
            continue
        opened = OPEN_RE.match(line)
        if not opened:
            continue
        flags = (opened.group("flags") or "").split()
        name = opened.group("name")
        rest = line[opened.end() :]
        # A short class definition (`package Medium = A.B;`, right-hand side
        # possibly on later lines) and a `redeclare` of one are declarations,
        # not scopes: neither has a matching `end`, so neither may be pushed.
        short = rest.lstrip().startswith("=") or "redeclare" in flags
        if not short:
            results.append((".".join(stack + [name]), opened.group("kind"), flags))
            if not line.rstrip().endswith(";"):
                stack.append(name)
    return results


def walk(directory: Path, prefix):
    """Walk a directory-form Modelica package."""
    results = []
    package = directory / "package.mo"
    if package.is_file():
        results += scan(package, prefix[:-1])
    for child in sorted(directory.iterdir()):
        if child.is_dir() and (child / "package.mo").is_file():
            results += walk(child, prefix + [child.name])
        elif child.suffix == ".mo" and child.name != "package.mo":
            results += scan(child, prefix)
    return results


def concrete_models(entries, namespace):
    return [
        name
        for name, kind, flags in entries
        if kind == "model" and "partial" not in flags and name.startswith(namespace)
    ]


def enumerate_targets(msl_root: Path):
    """The pinned frontier: what a Fluid-capable compiler has to get through."""
    modelica = msl_root / "Modelica 4.1.0"
    targets = []

    examples = walk(modelica / "Fluid" / "Examples", ["Modelica", "Fluid", "Examples"])
    for model in concrete_models(examples, "Modelica.Fluid.Examples"):
        targets.append(("Fluid.Examples", model))

    for package in FLUID_COMPONENT_PACKAGES:
        entries = scan(modelica / "Fluid" / f"{package}.mo", ["Modelica", "Fluid"])
        for model in concrete_models(entries, f"Modelica.Fluid.{package}"):
            targets.append((f"Fluid.{package}", model))

    media = scan(modelica / "Media" / "package.mo", ["Modelica"])
    for model in concrete_models(media, "Modelica.Media.Examples"):
        targets.append(("Media.Examples", model))

    return targets


def load_targets(path: Path):
    targets = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        group, model = line.split("\t")
        targets.append((group, model))
    return targets


# --------------------------------------------------------------------------
# Measurement
# --------------------------------------------------------------------------


def compile_model(options, model: str, cache_dir: Path):
    """Compile one model. Returns (outcome, seconds, clean log)."""
    command = [
        options.rumoca,
        "compile",
        str(Path(options.msl_root) / ENTRY_POINT),
        "--model",
        model,
        "--source-root",
        str(options.msl_root),
        "--cache-dir",
        str(cache_dir),
    ]
    started = time.monotonic()
    try:
        finished = subprocess.run(
            command, capture_output=True, text=True, timeout=options.timeout
        )
    except subprocess.TimeoutExpired as expired:
        partial = (expired.stdout or b"") + (expired.stderr or b"")
        if isinstance(partial, bytes):
            partial = partial.decode("utf-8", "replace")
        return "timeout", float(options.timeout), ANSI.sub("", partial)
    elapsed = time.monotonic() - started
    log = ANSI.sub("", finished.stdout + finished.stderr)
    if finished.returncode == 0:
        return "ok", elapsed, log
    # A negative status is a signal: the compiler died rather than refusing.
    if finished.returncode < 0:
        return f"signal:{-finished.returncode}", elapsed, log
    if "panicked at" in log or "RUST_BACKTRACE" in log:
        return "panic", elapsed, log
    if CODE.search(log):
        return "error", elapsed, log
    return "internal", elapsed, log


def first_diagnostic(log: str):
    """The first diagnostic code and its message, as one flat line."""
    match = CODE.search(log)
    if not match:
        for line in log.splitlines():
            text = line.strip(" │|")
            if text.strip():
                return "", text.strip()[:400]
        return "", ""
    code = match.group(1)
    # The message runs from the code to the start of the source snippet frame.
    tail = log[match.end() :]
    message = []
    for line in tail.splitlines():
        text = line.strip()
        if text.startswith("╭") or text.startswith("+-") or text.startswith("---"):
            break
        text = text.lstrip("│|").strip()
        if not text and message:
            break
        if text:
            message.append(text)
        if len(" ".join(message)) > 300:
            break
    return code, " ".join(message)[:400]


def phase_of(code: str) -> str:
    for length in (3, 2):
        prefix = code[:length]
        if prefix in PHASE_OF_PREFIX:
            return PHASE_OF_PREFIX[prefix]
    return "unknown"


# --------------------------------------------------------------------------
# Classification against the Fluid P0 gap taxonomy
# --------------------------------------------------------------------------
#
# Buckets, from the 2026-08-28 comm-channel tranche and the verdict on it:
#
#   p0a-specialization  redeclare / Medium selection: wrong or missing
#                       effective types, signature mismatch, alias fallback,
#                       depth-cap effects.
#   p0c-topology        connector array / inside-outside / scope failures.
#   p0d-stream          inStream / actualStream lowering failures.
#   p0e-balance         ED001-class wrong counts on Fluid shapes.
#   as051               the KNOWN Boolean-connector balance defect, which has
#                       its own fix in flight and must not be counted as new.
#   zero-extent         nXi=0 / nC=0 empty array or empty record failures.
#   p0g-annotations     derivative / inverse / smoothOrder relations.
#   other-known         defects already filed with repros (AS-041/042 etc.).
#   genuinely-new       does not map to any of the above.
#   UNCLASSIFIED        not yet examined. An honest bucket, never a guess.

# The mechanism citations name the exact compiler site the evidence convicts,
# so the scoreboard says which deletion each blocked model buys. Every site was
# confirmed present in the pinned compiler tree before being cited here:
#
#   ALIAS      rumoca-phase-instantiate/src/type_overrides/override_map.rs
#              -- an exact alias DefId falls back to a path key, missing
#              declarations are filter-mapped away, and a missing selected
#              package or member returns rather than refusing.
#   SIGWALK    rumoca-phase-typecheck/src/function_signatures.rs
#              -- parallel string / DefId / unqualified maps and a depth-64
#              walk whose overflow, cycle and missing cases continue.
#   FIXPOINT   rumoca-phase-flatten/src/constant_extraction.rs
#              -- bounded recovery loops, MAX_PASSES 4 and 5, so behaviour is
#              depth dependent: a structurally shallower sibling passes.
#   GUESSNAME  rumoca-phase-flatten/src/equations/mod.rs::lookup_parameter_in_scope
#              -- classifies a reference by initial CAPITALIZATION, retries
#              case-mangled spellings, and finally infers nX/nXi/nC/nS from
#              already-known array dimensions instead of consuming the fact.
#   STREAM     rumoca-phase-flatten/src/connections/stream_operators.rs

ALIAS = "ALIAS override_map.rs alias-path fallback"
SIGWALK = "SIGWALK function_signatures.rs depth-capped signature walk"
FIXPOINT = "FIXPOINT constant_extraction.rs bounded-pass recovery"
GUESSNAME = "GUESSNAME equations/mod.rs::lookup_parameter_in_scope"
STREAM = "STREAM connections/stream_operators.rs"

# A member type reached through a replaceable package slot. The Fluid families
# spell the slot `Medium`, `Medium_1`, `Medium_2`, `Medium1` or `BatchMedium`;
# a diagnostic naming `<slot>.<member>` is by construction a statement about
# effective specialization.
MEDIUM_MEMBER = re.compile(
    r"`(?:[\w.]*\.)?(?:Medium|Medium_?[12]|BatchMedium)\.[\w.]+`"
)

# The replaceable package SLOT itself, standing where a type should stand.
# `Medium.AbsolutePressure p` whose reported type is `...FluidPort.Medium` is
# the alias collapse reproduced in tools/rumoca-repros/medium-slot-as-type/:
# the member type is discarded and the package slot is kept in its place.
MEDIUM_SLOT_AS_TYPE = re.compile(
    r"`[\w.]*\.(?:Medium|Medium_?[12]|BatchMedium)`"
)

# A type named inside one of the partial Media/Fluid interface base classes.
# When the compiler reports one of these as the EFFECTIVE type of an occurrence
# the model redeclared to a concrete class, the selection did not take.
PARTIAL_BASE = re.compile(r"`[\w.]*\.Partial[A-Z]\w*\.[\w.]+`")

# A member read off a package-level constant of record type. Reproduced
# standalone in tools/rumoca-repros/package-record-constant/: every member
# access on such a constant is unresolved however the value is supplied, while
# a scalar package constant in the same package resolves.
RECORD_CONSTANT = re.compile(
    r"unresolved Flat reference `[\w.]*\b(?:data|Constants|fluidConstants)\."
)


def classify(code, message, log):
    """Return (bucket, evidence, mechanism) for one typed failure.

    Every rule states the evidence it matched on. A failure that matches
    nothing lands in UNCLASSIFIED, which is an honest bucket: an unexamined
    failure is more useful recorded as unexamined than filed under a guess.
    """

    # --- Balance: ED001 carries its own counts, so read them. -------------
    if code == "ED001":
        found = BALANCE.search(log)
        counts = (
            f"{found.group(1)} equations / {found.group(2)} unknowns"
            if found
            else "no counts in the diagnostic"
        )
        # AS-051 is the known Boolean-connector defect with a fix in flight.
        # It must never be counted as a new Fluid finding.
        if "Boolean" in log:
            return "as051", f"ED001 {counts}, Boolean connector member in scope", ""
        return "p0e-balance", f"ED001 {counts} on a Fluid shape", ""

    # --- Package-level record constants (reproduced standalone) ----------
    if code == "ED008" and RECORD_CONSTANT.search(message):
        return (
            "genuinely-new",
            "member read off a package-level constant of record type; "
            "reproduced standalone, where a scalar package constant resolves "
            "and every record-constant member does not",
            FIXPOINT,
        )

    # --- Effective specialization ----------------------------------------
    if code == "EI012" and "partial class" in message:
        return (
            "p0a-specialization",
            f"{code} instantiates the constraining/partial class for an "
            "occurrence the model redeclared to a concrete medium",
            ALIAS,
        )
    if MEDIUM_SLOT_AS_TYPE.search(message):
        return (
            "p0a-specialization",
            f"{code} reports the replaceable package slot itself as the type "
            "of a component declared with one of its member types; "
            "reproduced standalone",
            ALIAS,
        )
    if PARTIAL_BASE.search(message):
        return (
            "p0a-specialization",
            f"{code} names a partial interface class as the effective type of "
            "a redeclared occurrence",
            ALIAS,
        )
    if code == "EF019":
        return (
            "p0a-specialization",
            "resolved function reference is rendered as the partial base's "
            "member while its structured identity is a bare name",
            ALIAS,
        )
    if code == "EF024":
        member = message.split(":")[-1].strip()
        return (
            "p0a-specialization",
            "a member of the selected package (function, record constant or "
            f"structural constant `{member}`) reaches Flat with no structured "
            "identity",
            GUESSNAME,
        )
    if code == "EF025":
        return (
            "p0a-specialization",
            f"{code} ambiguous effective function selection under redeclare: "
            f"{message.split(':')[-1].strip()[:120]}",
            SIGWALK,
        )
    if code in ("EF015", "EF016", "EI007", "EI027"):
        return (
            "p0a-specialization",
            f"{code} on a redeclare/selection route: {message[:150]}",
            ALIAS,
        )
    if MEDIUM_MEMBER.search(message):
        return (
            "p0a-specialization",
            f"{code} names a selected-package member type; the identical "
            "declaration compiles in a shallow standalone model, so the "
            "failure is depth dependent, not shape dependent",
            FIXPOINT,
        )
    if code == "ED008":
        return (
            "p0a-specialization",
            "unresolved reference to a structural constant the selected "
            "package supplies through its extends modification",
            FIXPOINT,
        )

    # --- Zero extent ------------------------------------------------------
    # Reproduced standalone: a zero-extent array of a connector whose members
    # come from a replaceable package loses the member entirely, while the
    # same zero-extent array of a plain connector compiles.
    if code == "ET001" and "unknown member" in message:
        return (
            "zero-extent",
            "member lookup on a zero-extent connector array whose element "
            "type draws its members from a replaceable package; reproduced "
            "standalone, where the plain-connector zero-extent array compiles",
            GUESSNAME,
        )

    # --- Stream semantics -------------------------------------------------
    if re.search(r"inStream|actualStream", message):
        return "p0d-stream", f"{code} names stream lowering", STREAM

    # --- Connector topology ----------------------------------------------
    if re.search(r"\bconnect\b|connector|inside|outside", message, re.IGNORECASE):
        return "p0c-topology", f"{code} on a connection/connector route", ""

    # --- Declared-unsupported features, honestly reported -----------------
    if code == "ED019" and "MLS " in message:
        return (
            "declared-unsupported",
            f"{code} rejects a named MLS construct the compiler does not yet "
            "represent, rather than miscompiling it",
            "",
        )

    # --- Structural evaluator ---------------------------------------------
    if code == "ED019" and "cannot be evaluated" in message:
        return (
            "genuinely-new",
            "the structural evaluator fails on a parameter binding whose "
            "operands it has mistyped",
            FIXPOINT,
        )

    # --- Discrete solved form ---------------------------------------------
    if code == "ED010":
        return (
            "genuinely-new",
            "Appendix B discrete solved form rejects a StateGraph inner/outer "
            "or array-valued discrete equation; not a Fluid gap, but it blocks "
            "a Fluid example through its StateGraph dependency",
            "",
        )

    # --- Annotation-borne function relations ------------------------------
    if re.search(r"derivative|inverse|smoothOrder", message, re.IGNORECASE):
        return "p0g-annotations", f"{code} names a function-relation annotation", ""

    return "UNCLASSIFIED", f"{code} matched no taxonomy rule", ""


# --------------------------------------------------------------------------


def measure(options, targets, cache_dir: Path, logs_dir):
    rows = [None] * len(targets)

    def one(index):
        group, model = targets[index]
        outcome, elapsed, log = compile_model(options, model, cache_dir)
        if logs_dir:
            (logs_dir / f"{model}.log").write_text(log, encoding="utf-8")
        if outcome == "ok":
            return index, {
                "model": model,
                "package": group,
                "outcome": "ok",
                "phase": "",
                "code": "",
                "bucket": "",
                "seconds": f"{elapsed:.1f}",
                "diagnostic": "",
                "evidence": "",
                "mechanism": "",
            }
        code, message = first_diagnostic(log)
        if outcome in ("timeout", "panic", "internal") or outcome.startswith("signal:"):
            # Never folded into a typed failure: an untyped stop is its own row.
            bucket, evidence, mechanism = "", "", ""
            if outcome == "timeout":
                message = f"no verdict within {options.timeout}s"
                code = ""
        else:
            bucket, evidence, mechanism = classify(code, message, log)
        return index, {
            "model": model,
            "package": group,
            "outcome": outcome,
            "phase": phase_of(code) if code else "",
            "code": code,
            "bucket": bucket,
            "seconds": f"{elapsed:.1f}",
            "diagnostic": message,
            "evidence": evidence,
            "mechanism": mechanism,
        }

    with concurrent.futures.ThreadPoolExecutor(max_workers=options.jobs) as pool:
        futures = [pool.submit(one, index) for index in range(len(targets))]
        done = 0
        for future in concurrent.futures.as_completed(futures):
            index, row = future.result()
            rows[index] = row
            done += 1
            print(
                f"[{done:3}/{len(targets)}] {row['outcome']:8} "
                f"{row['code'] or '-':7} {row['model']}",
                file=sys.stderr,
                flush=True,
            )
    return rows


def reclassify(previous: Path, logs_dir: Path):
    """Re-bucket a finished run from its kept logs. No compiler is started.

    The OUTCOME of every row is carried over from the previous CSV unchanged:
    what the compiler did is the compiler's to say, and a reclassification must
    never be able to turn a failure into a pass. Only the bucket, the evidence
    and the mechanism -- the parts this rig authors -- are re-derived.
    """
    rows = []
    for row in csv.DictReader(previous.open(encoding="utf-8")):
        if row["outcome"] == "ok":
            rows.append(row | {"bucket": "", "evidence": "", "mechanism": ""})
            continue
        if row["outcome"] in ("timeout", "panic", "internal") or row[
            "outcome"
        ].startswith("signal:"):
            # An untyped stop is its own row and is never given a bucket.
            rows.append(row | {"bucket": "", "evidence": "", "mechanism": ""})
            continue
        log_path = logs_dir / f"{row['model']}.log"
        if not log_path.is_file():
            print(f"fluid-frontier: no log for {row['model']}", file=sys.stderr)
            rows.append(row)
            continue
        log = log_path.read_text(encoding="utf-8")
        code, message = first_diagnostic(log)
        bucket, evidence, mechanism = classify(code, message, log)
        rows.append(
            row
            | {
                "phase": phase_of(code) if code else "",
                "code": code,
                "bucket": bucket,
                "diagnostic": message,
                "evidence": evidence,
                "mechanism": mechanism,
            }
        )
    return rows


def write_rows(rows, out: Path):
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


FIELDS = [
    "model",
    "package",
    "outcome",
    "phase",
    "code",
    "bucket",
    "seconds",
    "diagnostic",
    "evidence",
    "mechanism",
]


def scoreboard(rows):
    """The per-package table and the headline number."""
    packages, buckets = {}, {}
    for row in rows:
        entry = packages.setdefault(row["package"], {"total": 0, "ok": 0, "buckets": {}})
        entry["total"] += 1
        if row["outcome"] == "ok":
            entry["ok"] += 1
            continue
        key = row["bucket"] or row["outcome"]
        entry["buckets"][key] = entry["buckets"].get(key, 0) + 1
        buckets[key] = buckets.get(key, 0) + 1

    lines = []
    total = sum(entry["total"] for entry in packages.values())
    compiling = sum(entry["ok"] for entry in packages.values())
    lines.append(f"{compiling} of {total} Fluid frontier models compile today.")
    lines.append("")
    lines.append(f"{'package':22} {'total':>5} {'ok':>4}  failing by bucket")
    for name in sorted(packages):
        entry = packages[name]
        detail = ", ".join(
            f"{bucket}={count}" for bucket, count in sorted(entry["buckets"].items())
        )
        lines.append(
            f"{name:22} {entry['total']:>5} {entry['ok']:>4}  {detail or '-'}"
        )
    lines.append("")
    lines.append("blocked models per gap (the campaign's measured priority order):")
    for bucket, count in sorted(buckets.items(), key=lambda item: -item[1]):
        lines.append(f"  {count:>4}  {bucket}")
    return "\n".join(lines)


def main(argv=None) -> int:
    options = parse_arguments(argv)
    here = Path(__file__).resolve().parent
    targets_path = Path(options.targets) if options.targets else here / "targets.tsv"

    if options.enumerate:
        targets = enumerate_targets(Path(options.msl_root))
        targets_path.write_text(
            "".join(f"{group}\t{model}\n" for group, model in targets), encoding="utf-8"
        )
        print(f"wrote {len(targets)} targets to {targets_path}")
        return 0

    if options.reclassify:
        previous = Path(options.out) if options.out else here / "frontier.csv"
        if not options.logs or not previous.is_file():
            print(
                "fluid-frontier: --reclassify needs --logs and an existing --out CSV",
                file=sys.stderr,
            )
            return 2
        rows = reclassify(previous, Path(options.logs))
        write_rows(rows, previous)
        print()
        print(scoreboard(rows))
        return 0

    if not Path(options.rumoca).exists():
        print(f"fluid-frontier: no such binary: {options.rumoca}", file=sys.stderr)
        return 2
    entry = Path(options.msl_root) / ENTRY_POINT
    if not entry.is_file():
        print(
            f"fluid-frontier: --msl-root is not a release layout: {entry} is missing",
            file=sys.stderr,
        )
        return 2

    targets = load_targets(targets_path)
    if options.only:
        wanted = set(options.only.split(","))
        targets = [target for target in targets if target[1] in wanted]
        if not targets:
            print("fluid-frontier: --only matched no target", file=sys.stderr)
            return 2

    out = Path(options.out) if options.out else here / "frontier.csv"
    out.parent.mkdir(parents=True, exist_ok=True)
    cache_dir = Path(options.cache_dir) if options.cache_dir else out.parent / ".cache"
    cache_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = None
    if options.logs:
        logs_dir = Path(options.logs)
        logs_dir.mkdir(parents=True, exist_ok=True)

    version = subprocess.run(
        [options.rumoca, "build-info"], capture_output=True, text=True
    ).stdout.strip()
    print(f"fluid-frontier: {len(targets)} targets, compiler {version}", file=sys.stderr)
    if not version.startswith(PIN):
        # Not a warning about correctness: a run against another binary is
        # exactly what this rig is for. It is a warning about comparison, so
        # nobody diffs the committed CSV against a different compiler and reads
        # the difference as a regression.
        print(
            f"fluid-frontier: this is not the pin {PIN}; the committed CSV was "
            "measured with the pin, so a diff against it mixes two compilers",
            file=sys.stderr,
        )

    rows = measure(options, targets, cache_dir, logs_dir)

    with out.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    report = scoreboard(rows)
    print()
    print(f"compiler: {version}")
    print(f"rows:     {out}")
    print()
    print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
