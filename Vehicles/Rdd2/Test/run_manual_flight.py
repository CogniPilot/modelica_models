#!/usr/bin/env python3
"""Fly the RDD2 manual-flight mission and let its own assertions judge it.

Vehicles.Rdd2.Test.ManualFlightMission carries its qualification claims as
equation-section assertions, so there is nothing for this script to evaluate:
Rumoca raises the moment one is violated and the exit status is the verdict.

The mission is simulated through the Python binding rather than the command
line because the command line writes a trace of every solver step of every
variable, which for a closed-loop mission is gigabytes of CSV and tens of
gigabytes of peak memory to format it. Nothing reads that trace. The binding
keeps the result in the worker process, which is released with it.
"""

from __future__ import annotations

import multiprocessing
from pathlib import Path
import sys

import rumoca as rum


SCENARIO = Path(__file__).with_name("rumoca-scenario.manual-flight.toml")
MISSION_DURATION_S = 28.0


def _simulate(scenario: str) -> None:
    """Compile and simulate the mission, discarding the trace."""
    _session, model, simulation = rum.Session.from_scenario(scenario)
    model.simulate(t=(0.0, MISSION_DURATION_S), config=simulation)


def main() -> int:
    print(f"simulating {SCENARIO.name}...", flush=True)
    # A spawned worker matches Vehicles/Rdd2/Test/run_waypoint_qualification.py:
    # it gives the mission a fresh JIT lifetime and releases the whole trace
    # when it exits, and it is portable to Windows and macOS.
    context = multiprocessing.get_context("spawn")
    with context.Pool(processes=1, maxtasksperchild=1) as pool:
        pool.apply(_simulate, (str(SCENARIO),))
    print(f"{SCENARIO.name} flew every assertion", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
