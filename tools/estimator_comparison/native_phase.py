"""Declare a bounded takeoff phase in the external DAL replay stream."""

import math


def flight_phase_writer(writer, clear_at_s):
    if clear_at_s is None:
        return writer
    if not math.isfinite(clear_at_s) or not 13 < clear_at_s <= 60:
        raise ValueError("Takeoff expectation must clear after arming and by 60 s")

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
