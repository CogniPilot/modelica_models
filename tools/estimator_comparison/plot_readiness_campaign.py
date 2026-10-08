"""Plot every matched horizontal-position pair, including ESKF losses."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, MaxNLocator


def plot(args):
    with args.comparison.open() as stream:
        rows = [row for row in csv.DictReader(stream) if row["window"] == "flight"]
    groups = {(row["capture"], row["estimator"], row["scenario"]): row for row in rows}
    if len(groups) != len(rows):
        raise ValueError("Duplicated flight comparison")
    captures = sorted({row["capture"] for row in rows})
    scenarios = ("gps", "denied", "transition")
    metric = "horizontal_position_rmse_m"
    figure, axes = plt.subplots(2, 3, figsize=(11, 7), constrained_layout=True)
    for row_index, native in enumerate(("px4", "ekf3")):
        for column, scenario in enumerate(scenarios):
            axis = axes[row_index, column]
            plotted = []
            for estimator, color, marker, label in (
                ("horizon", "#0072B2", "o", "ESKF horizon"),
                ("horizon_joint", "#D55E00", "+", "ESKF joint pressure horizon"),
            ):
                pairs = [
                    (
                        groups[capture, native, scenario],
                        groups[capture, estimator, scenario],
                    )
                    for capture in captures
                ]
                if any(
                    item[flag] != "True"
                    for pair in pairs
                    for item in pair
                    for flag in ("state_valid", "covariance_valid")
                ):
                    raise ValueError(
                        "Inspect invalid rows before plotting finite paired RMS"
                    )
                x, y = (
                    [float(pair[index][metric]) for pair in pairs] for index in (0, 1)
                )
                axis.scatter(
                    x, y, color=color, marker=marker, s=45, label=label, alpha=0.85
                )
                plotted.extend(x + y)
            lower, upper = 0, max(plotted) * 1.15
            axis.plot([lower, upper], [lower, upper], "--", color="0.5", linewidth=1)
            axis.set(xlim=(lower, upper), ylim=(lower, upper))
            for coordinate in (axis.xaxis, axis.yaxis):
                coordinate.set_major_locator(MaxNLocator(nbins=4))
                coordinate.set_major_formatter(FormatStrFormatter("%.2f"))
            axis.set_aspect("equal", adjustable="box")
            axis.grid(True, which="both", alpha=0.2)
            axis.set_title(
                {
                    "gps": "GPS",
                    "denied": "GPS denied",
                    "transition": "GPS loss / return",
                }[scenario]
            )
            axis.set_xlabel(
                ("PX4 EKF2" if native == "px4" else "ArduPilot EKF3")
                + " horizontal RMS (m)"
            )
            axis.set_ylabel("ESKF horizontal RMS (m)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="outside lower center", ncol=2, frameon=False)
    figure.suptitle(
        "Eight matched captures per panel; points below the diagonal favor ESKF"
    )
    args.output.mkdir(parents=True, exist_ok=False)
    for suffix in ("png", "pdf", "svg"):
        path = args.output / ("horizontal-pairs." + suffix)
        figure.savefig(path, dpi=180, bbox_inches="tight")
        if suffix == "svg":
            path.write_text(
                "\n".join(line.rstrip() for line in path.read_text().splitlines())
                + "\n"
            )
    plt.close(figure)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    plot(parser.parse_args())
