"""Verify DAL phase changes preserve independent flags and sensor messages."""

import unittest

from native_phase import flight_phase_writer


class Recorder:
    def __init__(self):
        self.messages = []

    def msg(self, name, *values):
        self.messages.append((name, values))


class FlightPhaseTests(unittest.TestCase):
    def test_timeout_changes_only_takeoff_and_emits_once(self):
        writer = flight_phase_writer(Recorder, 18)()
        payload = (40, -86, 200, 1, 200000, 0, 0, 0, 2, 3, 0b11010001)
        writer.msg("RFRH", 13_000_000, 0)
        writer.msg("RFRN", *payload)
        writer.msg("RFRH", 17_998_750, 4998)
        writer.msg("RISI", 1, 2, 3)
        self.assertEqual(len(writer.messages), 4)
        writer.msg("RFRH", 18_000_000, 5000)
        self.assertEqual(writer.messages[-1], ("RFRN", (*payload[:-1], 0b10010001)))
        writer.msg("RFRH", 18_001_250, 5001)
        self.assertEqual(sum(name == "RFRN" for name, _ in writer.messages), 2)
        writer.msg("RFRN", *payload)
        self.assertEqual(writer.messages[-1][1][-1], 0b10010001)
        self.assertIn(("RISI", (1, 2, 3)), writer.messages)

    def test_no_takeoff_flag_or_historical_mode(self):
        self.assertIs(flight_phase_writer(Recorder, None), Recorder)
        writer = flight_phase_writer(Recorder, 18)()
        writer.msg("RFRN", 0, 0b10001)
        writer.msg("RFRH", 18_000_000, 5000)
        self.assertEqual(len(writer.messages), 2)
        for value in (13, 0, 61, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                flight_phase_writer(Recorder, value)


if __name__ == "__main__":
    unittest.main()
