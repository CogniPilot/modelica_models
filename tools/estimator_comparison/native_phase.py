"""Declare a bounded takeoff phase in the external DAL replay stream."""

import math


def flight_phase_writer(writer, clear_at_s, arm_after_s=13):
    if clear_at_s is None:
        return writer
    if (
        not math.isfinite(arm_after_s)
        or arm_after_s < 0
        or not math.isfinite(clear_at_s)
        or not arm_after_s < clear_at_s <= arm_after_s + 47
    ):
        raise ValueError("Takeoff expectation must clear within the declared flight")

    class FlightPhaseWriter(writer):
        frame_time_us = 0
        flight = None

        def msg(self, name, *values):
            if name == "RFRH":
                self.frame_time_us = values[0]
            if name == "RFRN":
                values = list(values)
                if self.frame_time_us >= clear_at_s * 1e6:
                    values[-1] &= ~(1 << 6)
                self.flight = values
            super().msg(name, *values)
            if (
                name == "RFRH"
                and self.frame_time_us >= clear_at_s * 1e6
                and self.flight is not None
                and self.flight[-1] & (1 << 6)
            ):
                self.flight = self.flight.copy()
                self.flight[-1] &= ~(1 << 6)
                super().msg("RFRN", *self.flight)

    return FlightPhaseWriter
