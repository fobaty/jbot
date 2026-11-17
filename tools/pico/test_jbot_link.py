#!/usr/bin/env python3
"""Unit tests for the JBOT Pico console protocol layer.

Run from the repository root:

    python3 -m unittest discover -s tools/pico -p 'test_*.py'
"""

import os
import tempfile
import unittest

from jbot_link import (
    CsvLog,
    EventLog,
    Link,
    Telemetry,
    parse_stat,
    parse_telemetry,
)

SAMPLE = (
    "T step=1600 loop=200 imu=ok cal=0 pitch=41.58 gy=-0.12 err=0x00 mb=ok "
    "L=0 R=0 bat=41.50 tmp=39.3 age=0 estop=0"
)

STAT_SAMPLE = (
    "stat up=11.0 loop=199 stream=1 trim=0.00 estop=0 steps=2197 imu=1027 "
    "imu_err=0 imu_code=0 fb=0 fb_drop=17389"
)


class ParseTelemetryTest(unittest.TestCase):
    def test_parses_a_device_sample(self):
        telemetry = parse_telemetry(SAMPLE)
        self.assertIsInstance(telemetry, Telemetry)
        self.assertEqual(telemetry.step, 1600)
        self.assertEqual(telemetry.loop, 200)
        self.assertTrue(telemetry.imu_ok)
        self.assertTrue(telemetry.mainboard_ok)
        self.assertAlmostEqual(telemetry.pitch, 41.58, places=2)
        self.assertAlmostEqual(telemetry.gy, -0.12, places=2)
        self.assertEqual(telemetry.err, 0)
        self.assertAlmostEqual(telemetry.bat, 41.50, places=2)
        self.assertAlmostEqual(telemetry.tmp, 39.3, places=1)
        self.assertFalse(telemetry.stopped)

    def test_parses_estop_and_fault(self):
        line = SAMPLE.replace("err=0x00", "err=0x05").replace("estop=0", "estop=1")
        telemetry = parse_telemetry(line)
        self.assertEqual(telemetry.err, 5)
        self.assertTrue(telemetry.stopped)

    def test_age_text_hides_the_sentinel(self):
        self.assertEqual(parse_telemetry(SAMPLE).age_text, "0")
        line = SAMPLE.replace("age=0", "age=4294967295")
        self.assertEqual(parse_telemetry(line).age_text, "--")

    def test_ignores_non_telemetry(self):
        self.assertIsNone(parse_telemetry("ok ping"))
        self.assertIsNone(parse_telemetry("JBOT pico bridge 0.1 ready"))
        self.assertIsNone(parse_telemetry(""))

    def test_rejects_incomplete_sample(self):
        self.assertIsNone(parse_telemetry("T step=1 imu=ok"))
        self.assertIsNone(parse_telemetry("T step=x loop=200"))

    def test_csv_row_shape(self):
        telemetry = parse_telemetry(SAMPLE)
        row = telemetry.csv_row(host_time=12.5)
        fields = row.strip().split(",")
        self.assertEqual(len(fields), 15)
        self.assertEqual(fields[0], "12.500")
        self.assertEqual(fields[1], "1600")
        self.assertEqual(fields[7], "0")
        self.assertEqual(fields[8], "ok")


class ParseStatTest(unittest.TestCase):
    def test_parses_a_device_reply(self):
        statistics = parse_stat(STAT_SAMPLE)
        self.assertEqual(statistics["loop"], 199)
        self.assertEqual(statistics["stream"], 1)
        self.assertEqual(statistics["estop"], 0)
        self.assertAlmostEqual(statistics["up"], 11.0, places=1)
        self.assertAlmostEqual(statistics["trim"], 0.0, places=2)
        self.assertEqual(statistics["fb_drop"], 17389)

    def test_ignores_other_lines(self):
        self.assertIsNone(parse_stat("ok ping"))
        self.assertIsNone(parse_stat("T step=1"))
        self.assertIsNone(parse_stat(""))


class LinkParseTest(unittest.TestCase):
    def test_classifies_lines(self):
        link = Link("/dev/null")
        self.assertEqual(link._parse("ok stream 10"), ("ok", "ok stream 10"))
        self.assertEqual(link._parse("err bad"), ("err", "err bad"))
        self.assertEqual(link._parse("JBOT pico bridge 0.1 ready")[0], "banner")
        self.assertEqual(link._parse("???")[0], "garbage")
        kind, statistics = link._parse(STAT_SAMPLE)
        self.assertEqual(kind, "stat")
        self.assertEqual(statistics["steps"], 2197)
        self.assertEqual(link.stat["loop"], 199)
        kind, telemetry = link._parse(SAMPLE)
        self.assertEqual(kind, "telemetry")
        self.assertEqual(link.telemetry.step, 1600)

    def test_poll_is_safe_when_closed(self):
        link = Link("/dev/null")
        self.assertFalse(link.is_open)
        self.assertEqual(link.poll(), [])
        self.assertFalse(link.write("ping"))


class LoggingTest(unittest.TestCase):
    def test_event_log_keeps_ring_and_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "events.log")
            log = EventLog(path, capacity=3)
            for index in range(5):
                log.add("info", "event %d" % index)
            self.assertEqual(log.total, 5)
            self.assertEqual(len(log.entries), 3)
            self.assertEqual(log.entries[-1][2], "event 4")
            log.close()
            with open(path, encoding="utf-8") as handle:
                self.assertEqual(len(handle.readlines()), 5)

    def test_csv_log_writes_header_and_rows(self):
        telemetry = parse_telemetry(SAMPLE)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "telemetry.csv")
            log = CsvLog(path)
            log.write(telemetry, host_time=1.0)
            log.write(telemetry, host_time=2.0)
            self.assertEqual(log.rows, 2)
            log.close()
            with open(path, encoding="utf-8") as handle:
                lines = handle.readlines()
        self.assertTrue(lines[0].startswith("t_s,step,loop_hz"))
        self.assertEqual(len(lines), 3)


if __name__ == "__main__":
    unittest.main()
