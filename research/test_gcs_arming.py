"""Offline check of the ground station arming state machine.

The arming logic lives in ESP32 firmware and cannot be exercised without a board, but it
decides when the vehicle launches, so it should not go untested. This transcribes the
switch handling from gcs-firmware/src/main.cpp and drives it through the switch sequences
an operator actually produces, including the sloppy ones.

Constants are parsed out of main.cpp rather than copied, so the test fails if the firmware
values drift away from what it checks.

The defect this was written for: ARM is both switches, LAUNCH is the left one alone. Letting
go of the right switch first turns one gesture into the other, and the vehicle launched on
the way out of arming.
"""
import re
import unittest
from pathlib import Path

FIRMWARE = (Path(__file__).resolve().parents[1] / "gcs-firmware" / "src" / "main.cpp")


def firmware_constants():
    text = FIRMWARE.read_text(encoding="utf-8", errors="replace")
    out = {}
    for name in ("ARM_HOLD_MS", "LAUNCH_HOLD_MS", "KILL_HOLD_MS"):
        m = re.search(rf"{name}\s*=\s*(\d+)", text)
        if not m:
            raise AssertionError(f"{name} not found in {FIRMWARE.name}")
        out[name] = int(m.group(1))
    out["has_interlock"] = "launchNeedsRelease" in text
    return out


class Arming:
    """Transcription of the switch handling in main.cpp loop()."""

    IDLE, ARMED, FLYING = "IDLE", "ARMED", "FLYING"

    def __init__(self, arm_ms, launch_ms, kill_ms, interlock=True):
        self.arm_ms, self.launch_ms, self.kill_ms = arm_ms, launch_ms, kill_ms
        self.interlock = interlock
        self.state = self.IDLE
        self.both_start = self.left_start = self.right_start = 0
        self.arm_fired = self.launch_fired = self.kill_fired = False
        self.needs_release = False
        self.sent = []

    def step(self, now, left, right):
        both = left and right
        if self.interlock and not left and not right:
            self.needs_release = False

        if both:
            if self.both_start == 0:
                self.both_start = now
            if (not self.arm_fired and now - self.both_start >= self.arm_ms
                    and self.state == self.IDLE):
                self.sent.append("ARM")
                self.state = self.ARMED
                self.arm_fired = True
                if self.interlock:
                    self.needs_release = True
        else:
            self.both_start = 0
            self.arm_fired = False

        if left and not right and not (self.interlock and self.needs_release):
            if self.left_start == 0:
                self.left_start = now
            if (not self.launch_fired and now - self.left_start >= self.launch_ms
                    and self.state == self.ARMED):
                self.sent.append("LAUNCH")
                self.state = self.FLYING
                self.launch_fired = True
        else:
            self.left_start = 0
            self.launch_fired = False

        if right and not left:
            if self.right_start == 0:
                self.right_start = now
            if not self.kill_fired and now - self.right_start >= self.kill_ms:
                self.sent.append("KILL")
                self.state = self.IDLE
                self.kill_fired = True
        else:
            self.right_start = 0
            self.kill_fired = False
        return self.sent

    def run(self, segments, dt=10):
        """segments: list of (duration_ms, left, right). Returns commands in order."""
        now = dt
        for duration, left, right in segments:
            end = now + duration
            while now < end:
                self.step(now, left, right)
                now += dt
        return list(self.sent)


C = firmware_constants()


def machine(interlock=True):
    return Arming(C["ARM_HOLD_MS"], C["LAUNCH_HOLD_MS"], C["KILL_HOLD_MS"], interlock)


class FirmwareConstantsTest(unittest.TestCase):
    def test_interlock_is_present(self):
        self.assertTrue(C["has_interlock"],
                        "main.cpp no longer contains the launchNeedsRelease interlock")

    def test_arm_and_launch_are_deliberate(self):
        self.assertGreaterEqual(C["ARM_HOLD_MS"], 1500)
        self.assertGreaterEqual(C["LAUNCH_HOLD_MS"], 1500)

    def test_kill_stays_fast(self):
        """An emergency stop behind a long hold is not an emergency stop."""
        self.assertLessEqual(C["KILL_HOLD_MS"], 300)
        self.assertLess(C["KILL_HOLD_MS"], C["ARM_HOLD_MS"])


class ReleaseOrderTest(unittest.TestCase):
    """The sequences an operator actually produces when letting go after arming."""

    def test_releasing_right_first_does_not_launch(self):
        m = machine()
        out = m.run([(C["ARM_HOLD_MS"] + 100, True, True),   # arm
                     (C["LAUNCH_HOLD_MS"] + 500, True, False),  # fumble: right off first
                     (200, False, False)])
        self.assertIn("ARM", out)
        self.assertNotIn("LAUNCH", out)

    def test_old_behaviour_did_launch(self):
        """Without the interlock the same fumble launches. Keeps the defect documented."""
        m = machine(interlock=False)
        out = m.run([(C["ARM_HOLD_MS"] + 100, True, True),
                     (C["LAUNCH_HOLD_MS"] + 500, True, False),
                     (200, False, False)])
        self.assertIn("LAUNCH", out)

    def test_releasing_left_first_disarms_rather_than_launching(self):
        m = machine()
        out = m.run([(C["ARM_HOLD_MS"] + 100, True, True),
                     (C["KILL_HOLD_MS"] + 100, False, True),
                     (200, False, False)])
        self.assertNotIn("LAUNCH", out)
        self.assertEqual(m.state, Arming.IDLE)

    def test_clean_release_then_launch_works(self):
        m = machine()
        out = m.run([(C["ARM_HOLD_MS"] + 100, True, True),
                     (300, False, False),                       # clean release
                     (C["LAUNCH_HOLD_MS"] + 100, True, False)])
        self.assertEqual(out, ["ARM", "LAUNCH"])
        self.assertEqual(m.state, Arming.FLYING)


class HoldTimeTest(unittest.TestCase):
    def test_short_press_does_not_arm(self):
        m = machine()
        self.assertEqual(m.run([(C["ARM_HOLD_MS"] - 200, True, True),
                                (200, False, False)]), [])

    def test_short_left_press_does_not_launch(self):
        m = machine()
        out = m.run([(C["ARM_HOLD_MS"] + 100, True, True), (300, False, False),
                     (C["LAUNCH_HOLD_MS"] - 200, True, False), (200, False, False)])
        self.assertNotIn("LAUNCH", out)


class KillTest(unittest.TestCase):
    def test_kill_available_from_idle(self):
        m = machine()
        self.assertIn("KILL", m.run([(C["KILL_HOLD_MS"] + 50, False, True)]))

    def test_kill_available_in_flight(self):
        m = machine()
        m.run([(C["ARM_HOLD_MS"] + 100, True, True), (300, False, False),
               (C["LAUNCH_HOLD_MS"] + 100, True, False)])
        self.assertEqual(m.state, Arming.FLYING)
        m.run([(200, False, False), (C["KILL_HOLD_MS"] + 50, False, True)])
        self.assertIn("KILL", m.sent)
        self.assertEqual(m.state, Arming.IDLE)

    def test_kill_is_never_interlocked(self):
        """Straight from arming, with no clean release, KILL must still fire."""
        m = machine()
        m.run([(C["ARM_HOLD_MS"] + 100, True, True)])
        self.assertEqual(m.state, Arming.ARMED)
        m.run([(C["KILL_HOLD_MS"] + 50, False, True)])
        self.assertIn("KILL", m.sent)

    def test_launch_cannot_fire_from_idle(self):
        m = machine()
        self.assertNotIn("LAUNCH", m.run([(C["LAUNCH_HOLD_MS"] + 500, True, False)]))


if __name__ == "__main__":
    unittest.main()
