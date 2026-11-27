"""Plot the fixed-timestep Iowa propeller study from its published flow results."""
import csv
from pathlib import Path

import matplotlib.pyplot as plt

DOCS = Path(__file__).resolve().parents[1] / "docs"


def main():
    with (DOCS / "propeller-mesh-flow.csv").open() as source:
        # Only the original CAD runs at J=0.8. The R24 nominal timestep is
        # also the common physical timestep used in the fixed-step series.
        runs = sorted(
            (row for row in csv.DictReader(source)
             if float(row["J"]) == 0.8 and int(row["steps_per_turn"]) == 1006
             and (row["run"].endswith("-fixed") or row["run"] == "R24-J08")),
            key=lambda row: int(row["cells_per_D"]),
        )
    fig, (loads, costs) = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
    reference = float(runs[0]["IIHR_KT"])
    loads.axhline(reference, color="#3e525b", linestyle="--", label="IIHR measurement")
    accepted = []
    for row in runs:
        grid = int(row["cells_per_D"])
        kt = float(row["final_KT"])
        settled = row["settled"] == "True"
        color = "#117a72" if settled else "#8e6157"
        mark = "o" if settled else "x"
        loads.scatter(grid, kt, marker=mark, c=color)
        if settled:
            # Domain dimensions are (9R, 4R, 4R). Each time step updates all cells.
            updates = 144 * grid**3 * int(row["steps_per_turn"]) * int(row["periods"])
            accepted.append((updates / 1e9, 100 * abs(kt / reference - 1), grid))
    best_error = float("inf")
    frontier = []
    for work, err, grid in sorted(accepted):
        if err < best_error:
            frontier.append((work, err))
            best_error = err
        costs.scatter(work, err, color="#117a72", zorder=3)
        costs.annotate(f"{grid}/D", (work, err), xytext=(5, 5), textcoords="offset points")
    if frontier:
        costs.plot(*zip(*frontier), color="#117a72", linestyle=":", label="Observed frontier")
    for settled, color, mark in [(True, "#117a72", "o"), (False, "#8e6157", "x")]:
        loads.scatter([], [], c=color, marker=mark, label="Settled" if settled else "32-turn limit")
    loads.set(xlabel="Grid cells per diameter (finer →)", ylabel="Final turn-mean thrust coefficient",
              title="Thrust across grids", xticks=[int(r["cells_per_D"]) for r in runs])
    costs.set(xlabel="Billion cell updates to settle", ylabel="Absolute thrust error vs IIHR (%)",
              title="Accuracy against computational work")
    costs.margins(x=.18, y=.25)
    for ax in (loads, costs):
        ax.grid(alpha=.20)
        ax.legend(fontsize=8)
    fig.suptitle("Iowa port propeller · 538 rpm · J = 0.8 · 1,006 time steps per turn")
    fig.savefig(DOCS / "propeller-thrust-frontier.png", dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    main()
